#include "GeneratedAutons/RedHard.h"
#include "Commands/Rotate.h"
#include "Commands/Swing.h"
#include "Commands/InstantCommand.h"
#include "Commands/tank_motion_profile.hpp"
#include "CommandScheduler/Parallel.h"
#include "Telemetry/telemetry.h"
#include "util/timer.h"
#include "util/mathUtils.h"
#include <cmath>
#include <format>
#include <functional>
#include <set>

bool redhard_force_pin_miss = false;

namespace {

//red frame is the source of truth, blue is the whole field rotated 180 deg about the center: (x, y, heading) -> (-x, -y, heading + 180)
struct Frame {
    bool blue;
    double x(double rx) const { return blue ? -rx : rx; }
    double y(double ry) const { return blue ? -ry : ry; }
    double h(double deg) const {
        double a = std::fmod(deg + (blue ? 180.0 : 0.0), 360.0);
        if(a > 180.0) a -= 360.0;
        if(a <= -180.0) a += 360.0;
        return a;
    }
};

//--- MECHANISM PLACEHOLDERS (robot is drivetrain only today) ---
//Each one is a timed no-op that logs its name. Replace the body when the intake/lift exists.
//"hold" pieces need the robot stationary, "settle" pieces only stow/retract and are run beside the next chassis move.
class MechStep : public Command {
    private:
    const char* name;
    Timer timer;

    public:
    MechStep(const char* name, uint32_t ms) : name(name), timer(ms) {}
    void initialize() override {
        timer.reset();
        TELEMETRY.send(Telemetry::Channel::Debug, std::format(
            "{{\"t\": {}, \"event\": \"mech_step\", \"name\": \"{}\"}}\n", pros::millis(), name));
    }
    bool isFinished() override { return timer.isDone(); }
};

//place_pin: Pin held red-half-up, front edge ~4.5in from the Goal center, aimed at it, 0.25s hold + 0.15s retract
Command* placePinHold()      { return new MechStep("place_pin_hold", 250); }
Command* placePinSettle()    { return new MechStep("place_pin_settle", 150); }
//pick_pin: pin axis within +-2.5in of the intake axis, pin 9-14in ahead of the tracking center, 0.30s hold + 0.20s index
Command* pickPinHold()       { return new MechStep("pick_pin_hold", 300); }
Command* pickPinIndex()      { return new MechStep("pick_pin_index", 200); }
Command* pickPinFull()       { return new MechStep("pick_pin_full", 500); } //fallback grab, nothing to overlap with
//pick_cup: cup center within +-2.5in of the grabber axis, 9-14in ahead, high clamp so floor pins are not touched, 0.35s hold + 0.25s stow
Command* pickCupClamp()      { return new MechStep("pick_cup_clamp", 350); }
Command* pickCupStow()       { return new MechStep("pick_cup_stow", 250); }
//place_stack: cup (pin on top) lowered over the Goal's pin, front edge ~4.5in from the Goal center, 0.45s release + 0.25s retract
Command* placeStackRelease() { return new MechStep("place_stack_release", 450); }
Command* placeStackRetract() { return new MechStep("place_stack_retract", 250); }
//flip_toggle: front 6-8in from the toggle face, centered on it within +-4in, robot must be clear of it afterwards (SC4b)
Command* flipToggle(bool blue) { return new MechStep(blue ? "flip_toggle_blue" : "flip_toggle_red", 800); }

//stand-in for the intake's pin sensor, true while a Pin is held
bool pinHeld() { return !redhard_force_pin_miss; }

//Picks one of two branches when it starts (not before), used for the ghost-opponent fallback
class BranchCommand : public Command {
    private:
    std::function<bool()> cond;
    Command* onTrue;
    Command* onFalse;
    Command* chosen = nullptr;

    public:
    BranchCommand(std::function<bool()> cond, Command* onTrue, Command* onFalse) : cond(cond), onTrue(onTrue), onFalse(onFalse) {}
    void initialize() override {
        chosen = cond() ? onTrue : onFalse;
        chosen->initialize();
    }
    void execute() override { chosen->execute(); }
    bool isFinished() override { return chosen == nullptr || chosen->isFinished(); }
    void end(bool interrupted) override { if(chosen != nullptr) chosen->end(interrupted); }
    std::vector<Subsystem*> getRequirements() override {
        std::set<Subsystem*> subs;
        for(Command* c : {onTrue, onFalse}){
            for(Subsystem* s : c->getRequirements()) subs.insert(s);
        }
        return std::vector<Subsystem*>(subs.begin(), subs.end());
    }
};

//every chassis step gets a timeout of 1.25x its planned time (rounded up to 0.05s), planned times are in RedHard_NOTES.md
double budget(double planned) { return std::ceil(planned * 1.25 * 20.0) / 20.0; }

Command* swingTo(drivetrain* c, double deg, DriveSide side, double planned) { return c->swing(deg, side, false, 127, 0, 0, budget(planned)); }
Command* turnTo(drivetrain* c, double deg, double planned) { return c->rotate(deg, 0, budget(planned)); }
Command* faceTo(drivetrain* c, double x, double y, double planned) { return c->rotate_to_point(x, y, 0, false, budget(planned)); }
Command* driveTo(drivetrain* c, double x, double y, double planned) { return c->Tank_motion_profile(x, y, 0, Speed::NORMAL, budget(planned)); }
Command* backUp(drivetrain* c, double inches, double planned) { return c->moveForward(inches, true, 60, 0, budget(planned)); }

//runs the mechanism piece and the chassis step together. Both live inside one Parallel inside the one Sequence that owns the
//chassis, so the scheduler never sees a second command asking for the drivetrain and CancelRunning never fires. The Parallel
//ends when BOTH are done, so a chassis timeout can not cut the mechanism piece short.
Command* alongside(Command* mech, Command* chassisStep) { return new Parallel({mech, chassisStep}); }

Sequence* buildHard(drivetrain* c, bool blue, bool setStartPose) {
    Frame f{blue};

    //start: touching bottom wall, back edge on it, heading +Y. x chosen so the robot clears the Load Zone tape and the first wall Cup
    if(setStartPose) c->setPose(f.x(-41.5), f.y(-62.0), degToRad(f.h(90)));

    //ghost opponent: the line pin on C's left petal is gone, so take C's bottom petal instead (about +1.7s)
    Sequence* fallback = new Sequence({
        turnTo(c, f.h(25), 0.34),
        driveTo(c, f.x(12.0), f.y(-33.5), 0.44),
        pickPinFull(),
        driveTo(c, f.x(10.5), f.y(-30.5), 0.43)
    });

    return new Sequence({
        //1) Preload into R' (own red Goal at -22.5,-45). Swing about the right wheels so the back corner leaves the wall without spinning
        swingTo(c, f.h(44.1), DriveSide::RIGHT, 0.64),
        driveTo(c, f.x(-33.22), f.y(-51.43), 0.65),
        faceTo(c, f.x(-22.5), f.y(-45.0), 0.28),
        placePinHold(),
        alongside(placePinSettle(), backUp(c, 2.1, 0.37)),

        //2) Out through the 16.7in gap between cup Y_a and R', then east along y=-33.1 between R' and the Midfield vertex cup
        turnTo(c, f.h(90), 0.45),
        driveTo(c, f.x(-35.0), f.y(-33.1), 0.91),
        turnTo(c, f.h(0), 0.58),
        driveTo(c, f.x(9.5), f.y(-33.1), 1.34),

        //3) Autonomous Line cross C (22.5,-22.5): left petal pin first (heading 55), then the cup (heading 35), same spot
        faceTo(c, f.x(10.5), f.y(-30.5), 0.50),
        driveTo(c, f.x(10.5), f.y(-30.5), 0.40),
        turnTo(c, f.h(55), 0.29),
        pickPinHold(),
        new BranchCommand(pinHeld, new InstantCommand([]{}), fallback),
        alongside(pickPinIndex(), turnTo(c, f.h(35), 0.30)),
        pickCupClamp(),

        //4) Cup + pin onto N4 (neutral Goal at 22.5,-45) from the north-west
        alongside(pickCupStow(), faceTo(c, f.x(22.5), f.y(-45.0), 0.56)),
        driveTo(c, f.x(14.53), f.y(-35.37), 0.56),
        placeStackRelease(),
        alongside(placeStackRetract(), backUp(c, 6.5, 0.57)),

        //5) Bottom toggle last: nothing else needs the robot near the wall and it must end clear of the toggle
        faceTo(c, f.x(0.0), f.y(-55.5), 0.47),
        driveTo(c, f.x(0.0), f.y(-55.5), 1.05),
        turnTo(c, f.h(270), 0.32),
        flipToggle(blue)
    });
}

} //namespace

Sequence* RedHard(drivetrain* chassis, bool setStartPose) { return buildHard(chassis, false, setStartPose); }
Sequence* BlueHard(drivetrain* chassis, bool setStartPose) { return buildHard(chassis, true, setStartPose); }
