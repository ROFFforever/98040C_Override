#include "main.h"
#include "RobotConfig.h"
#include "Commands/TeleopCommands/LiftTeleopCommand.h"
#include "Commands/TeleopCommands/ClawTeleopCommand.h"
#include "Subsystems/Claw.h"
#include "Units.h"
#include "Subsystems/drivetrain.h"
#include "Telemetry/telemetry.h"
#include "CommandScheduler/commandScheduler.h"
#include "Commands/TeleopCommands/ArcadeDriveCommand.h"
#include "Commands/TeleopCommands/IntakeTeleopCommand.h"
#include "Subsystems/Intake.h"
#include "Controllers/PID.hpp"
#include "Controllers/velocity_feed_forward.hpp"
#include "util/mathUtils.h"
#include <cfenv>
#include <cstdio>
#include "Commands/Tuning/DriveCharacterize.h"
#include "Commands/Tuning/RawDrive.h"
#include "Commands/Tuning/CameraCharacterize.h"
#include "Telemetry/telemetry.h"
#include "Commands/Tuning/FeedForwardTest.h"
#include "Commands/Rotate.h"
#include "Commands/Swing.h"
#include "Commands/tank_motion_profile.hpp"
#include "Subsystems/Lift.h"
#include "Commands/LiftMoveTo.h"
#include "Commands/Tuning/RawTurn.h"
#include "Commands/WaitCommand.h"
#include "Autons/SoloAWP.h"
#include "Autons/OdomAccuracyTest.h"
#include "GeneratedAutons/RedHard.h"
#include "GeneratedAutons/auton_1.h"
#include "GeneratedAutons/skills.h"
#include "GeneratedAutons/skills_pt_2.h"
#include "GeneratedAutons/skills_pt_3.h"
#include "GeneratedAutons/skills_optimized_test.h"
#include <format>
#include "Subsystems/piston.h"
#include "Commands/Tuning/AngularCharacterize.h"
#include "Commands/TeleopCommands/PistonTeleopCommand.h"
#include "Subsystems/WallSensor.h"
#include "util/OdomScreen.h"
#include "Commands/Localization/WallLocalization.h"


#include "Commands/Tuning/AngularPIDTune.h"   // add near your other Commands includes
#include "Commands/Tuning/LateralPIDTune.h"
#include "Commands/Tuning/RotateDialTest.h"
#include "Commands/Tuning/MoveToPointDialTest.h"
#include "Commands/Tuning/LateralMotionDiagnostic.h"

bool wall_enabled=true;

#ifdef ROBOT_MAIN
pros::MotorGroup leftMotors({2, -1, -3});   // port numbers; negative = reversed
pros::MotorGroup rightMotors({8, -9, 10});
pros::Imu imu(12);
pros::Rotation vertRotation(20); //reverse angle
pros::Rotation horizRotation(11); //reverse angle
odom_wheel vert(&vertRotation, -1.125, 2.02); //1 1/8 right of center, 4.5 back (back distance unused)
odom_wheel horiz(&horizRotation, 0.75, 2.75); //0.75 back of center, 5/8 left (left distance unused)

//controllers
PID residual_lateral_PID(1.0 * 1000,0,45*100,0,0); //PID tuning run 3 (run_35 P-only 0.95: 2.6in overshoot; run_37 kP 1.0 kD 20: 1.76in overshoot). Was kP 0.95, kD 26 (tuned with the old 1.55V kS)
//PID residual_lateral_PID(0 * 1000,0,0*100,0,0);
PID angular_pid(26 * 1000.0,0,1800 * 100,0,0, 7000); //angular tuning (run_50 P-only: 31deg overshoot, 5 swings). Was kP 26, kD 1457
//avg of kav_trial_2 + kav_trial_3 (new drivetrain, refit from the vert pod). Old: kV 0.1384, kA 0.024, kS 1.551
velocity_feed_forward ff_lateral( 0.152 * 1000,  // kV
                          0.010 * 1000, // kA is 0 because it doens't pull much weight
                          0.39 * 1000);  // kS
						


velocity_feed_forward ff_angular(0.931 *1000,  // kV
0.1025*1000, // kA
1.91 * 1000);  // kS


#ifdef HAS_LIFT
pros::Motor lift_1(21);
//TODO TUNE: placeholder gains. Error is in degrees, output is mV. Add &lift_rotation as the 3rd Lift arg once the rotation sensor is installed
PID lift_pid(1200, 0, 0, 0, 0);
Lift lift({{&lift_1, false}}, &lift_pid);

pros::Motor claw_oc(7);
pros::Motor claw_suction(4);
PID claw_pid(300, 0, 0, 0, 0);               // TODO TUNE
Claw claw({{&claw_oc, false}}, {{&claw_suction, true}}, &claw_pid);


#endif

#else // ROBOT_TEST
pros::MotorGroup leftMotors({18, 20});   // port numbers; negative = reversed
pros::MotorGroup rightMotors({-11, -12});
// pros::MotorGroup leftMotors({-1, 2,-3});   // port numbers; negative = reversed
// pros::MotorGroup rightMotors({4, -5,6});
pros::Imu imu(13);
pros::Rotation vertRotation(-16); //reverse angle
pros::Rotation horizRotation(15); //reverse angle
odom_wheel vert(&vertRotation, 1.25, 2.03); //was 2.125, read ~5% long vs camera
odom_wheel horiz(&horizRotation, 0.75, 2.125);

//controllers
PID residual_lateral_PID(2.4 * 1000,0,70*100,0,0);
PID angular_pid(15.2 * 1000.0,0,117 * 1000,0,0, 9000); //9V raw cap, gains were tuned at 9V (12V hopped, run_64/66)


// //
// {"kS": 0.671754342335145, "kV": 0.13186696277648247, "kA": -0.0006958734869371364, "r2": 0.803989358366047, "rmse": 2.7154202310992996, "n": 1216, "vRange": [-123.13297872820009, 193.5721178570053], "aRange": [-19357.211785700532, 19357.211785700532]}
velocity_feed_forward ff_lateral(0.150 * 1000,  // kV
                          0.014 * 1000, // kA
                          1.0 * 1000);  // kS

velocity_feed_forward ff_angular(0.8995599372169607*1000,  // kV
0.089*1000, // kA
1.071431989714771 * 1000);  // kS

// Test robot currently has no intake, claw piston, or lift installed - just drivetrain.
#endif

pros::Controller controller(pros::E_CONTROLLER_MASTER);

#ifdef HAS_LIFT
//the claw and lift macros fire each other, so liftTeleop is declared here and defined below
extern LiftTeleopCommand liftTeleop;
Command* lift_to_430 = lift.moveTo(-430); //fired by claw button L1
//claw macros: one button per position
ClawTeleopCommand clawTeleop(&claw, &controller, {
	{pros::E_CONTROLLER_DIGITAL_L1, claw.moveTo(ClawPos::CLOSED), true, []{ liftTeleop.trigger(lift_to_430); }, 300}, //true = stays braked once closed so the cone stays in place. Lift starts 300ms after the press
	{pros::E_CONTROLLER_DIGITAL_L2, claw.moveTo(ClawPos::OPEN)},
}, 50); //suction spins at 50% while closed(L1), stops on open(L2)
//used by lift button B once its move finishes: opens the claw 30 degrees past wherever it currently is(positive = opens)
Command* claw_open_small = claw.moveBy(30);
//lift macros: one button per position
LiftTeleopCommand liftTeleop(&lift, &controller, {
	{pros::E_CONTROLLER_DIGITAL_A, lift.moveTo(LiftPos::DOWN), false}, //false = coasts at the bottom instead of holding
	{pros::E_CONTROLLER_DIGITAL_B, lift.moveBy(500), true, []{ clawTeleop.trigger(claw_open_small, true); }}, //500 down from wherever the lift is(positive = down), then once that finishes opens the claw 30 more and holds it there
	{pros::E_CONTROLLER_DIGITAL_X, lift.moveTo(-530)},
	{pros::E_CONTROLLER_DIGITAL_Y, lift.moveTo(LiftPos::FIRST_CONE)},
	{pros::E_CONTROLLER_DIGITAL_DOWN, lift.moveTo(LiftPos::SECOND_CONE)},
	{pros::E_CONTROLLER_DIGITAL_LEFT, lift.moveTo(LiftPos::THIRD_CONE)},
	{pros::E_CONTROLLER_DIGITAL_UP, lift.moveTo(LiftPos::FOURTH_CONE)},
	{pros::E_CONTROLLER_DIGITAL_RIGHT, lift.moveTo(LiftPos::FIFTH_CONE)},
});
#endif
#ifdef ROBOT_MAIN

#endif


//vertOffset was 8.625, then 7.2. 7.2 was measured from the camera's point, which sat ~0.8in behind the odom
//tracking point (tag profile fixed 10/16). Measured from the odom point it's 7.7-7.8
//sensor's measuring point sits 5/8in further toward robot center than CAD says, so every vertOffset moves 5/8in inward
constexpr float kWallZeroInset = 0.625f;
WallSensor back_wall_sensor(17, -5.627f, -0.131f - kWallZeroInset,  WallSensor::Side::BACK); //5.627 right, 0.131 forward of center

//vertOffset=0.75  horizOffset=7.26
//vertOffset was 6.875; 5.85 from the camera fit on the 10/16 runs (read ~1in long)
WallSensor left_wall_sensor(13, 2.814f, 4.968f - kWallZeroInset,  WallSensor::Side::LEFT); //4.968 left, 2.814 forward of center (CAD)

// vertOffset=7.30  horizOffset=1.80
WallSensor front_wall_sensor(14, -6.256f, 0.39f,  WallSensor::Side::FRONT); //6.076 left, 1.25 forward of center


//vertOffset was 6.50; 5.80 from the camera fit on the 10/16 runs (read ~0.7in long)
WallSensor right_wall_sensor(18, -2.512f, 4.182f,  WallSensor::Side::RIGHT); //4.916 right, 2.184 forward of center (CAD)


drivetrain chassis(&leftMotors, &rightMotors, &imu, Units::WHEEL_325, 720, &vert, &horiz, &angular_pid, &ff_lateral, &ff_angular, &residual_lateral_PID); // 450 = wheel's actual output rpm after gearing //720 measured: motor distance matched the vert pod on a push test + kAV trial 1 (60:36 on 200rpm math gives 333 but reads ~2.1x short)
WallLocalization wallLocalization({&back_wall_sensor, &left_wall_sensor, &front_wall_sensor, &right_wall_sensor}, &chassis);
ArcadeDriveCommand arcadeDrive(&chassis, &controller); // drivetrain's default teleop command
OdomScreen odomScreen(&chassis); // x/y/heading + raw odom wheels on the brain screen

//logs tuning values so run analysis knows what this build used. add new tuning values here too
void logRobotConfig(){
#ifdef ROBOT_MAIN
	const char* robot = "MAIN";
#else
	const char* robot = "TEST";
#endif
	auto ang = angular_pid.get_consts();
	auto lat = residual_lateral_PID.get_consts();
	auto ffl = ff_lateral.get_consts();
	auto ffa = ff_angular.get_consts();
	TELEMETRY.send(Telemetry::Channel::Debug, std::format(
		"{{\"t\": {}, \"event\": \"config\", \"robot\": \"{}\", \"wallEnabled\": {}, \"imuPort\": {}, "
		"\"vertOdom\": {{\"present\": {}, \"offset\": {}, \"diameter\": {}}}, "
		"\"horizOdom\": {{\"present\": {}, \"offset\": {}, \"diameter\": {}}}, "
		"\"angularPID\": [{}, {}, {}], \"lateralPID\": [{}, {}, {}], "
		"\"ffLateral\": {{\"kV\": {}, \"kA\": {}, \"kS\": {}}}, \"ffAngular\": {{\"kV\": {}, \"kA\": {}, \"kS\": {}}}, "
		"\"angularKS\": {}, \"lateralKS\": {}, {}}}\n",
		pros::millis(), robot, wall_enabled, imu.get_port(),
		vert.odom_sensor != nullptr, vert.offset, vert.wheel_diameter,
		horiz.odom_sensor != nullptr, horiz.offset, horiz.wheel_diameter,
		ang[0], ang[1], ang[2], lat[0], lat[1], lat[2],
		ffl[0], ffl[1], ffl[2], ffa[0], ffa[1], ffa[2],
		chassis.angular_kS, chassis.lateral_kS, wallLocalization.config_json()));
}


//In order to sync run video with program runtime data, we send a number wirelessly over telemetry.
//This number tells the computer what to name the video file so the right run + video can be synced
//specifically this function reads a file in the sd card in order to track what run it should be.
int nextRunId() {
	int id = 0;
	if (pros::usd::is_installed()) {
		std::FILE* f = std::fopen("/usd/run_id.txt", "r");
		if (f) {
			std::fscanf(f, "%d", &id);
			std::fclose(f);
		}
	}
	id += 1;
	if (pros::usd::is_installed()) {
		std::FILE* f = std::fopen("/usd/run_id.txt", "w");
		if (f) {
			std::fprintf(f, "%d", id);
			std::fclose(f);
		}
	}
	return id;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	//infinite loop which sends a heartbeat to the camera recording software.
	//once the heartbeat stops for a few seconds, it stops recording.

	//TODO UNDO THIS
	// pros::Task([]{
	// 	while (true) {
	// 		printf("heartbeat\n");
	// 		fflush(stdout);
	// 		pros::delay(500);
	// 	}
	// }, "Heartbeat");

	//lift always starts physically down, so zero it here. Motor encoders only reset when the motor loses power,
	//not when the program restarts, so without this the 0 point would be wherever it was last left.
#ifdef HAS_LIFT
	lift.resetPosition();
	lift.setHold(false); //starts physically down, so coast like the DOWN macro does. Every other macro holds when it finishes
	lift.setDebugStream(true); //set true to stream lift position over pros terminal every 0.5s
	claw.resetPosition(ClawPos::START); //claw starts every match fully open, so that physical position reads 0
	claw.setHold(true); //holds position at the start. After a claw macro finishes, ClawTeleopCommand switches it(only closed holds, everything else coasts)
	claw.brake();
	odomScreen.setLiftGetter([]{ return lift.getAngle(); }); //shows lift position on the brain screen
#endif

	int runId = nextRunId();
	//a message for the recording script to start(provides run ID)
	printf("RUN_ID:%d\n", runId);
	fflush(stdout);

	odomScreen.start(); //replaces the old pros::lcd screen

	//Sets mode to SD card based logging. It's enables faster writes, is more reliable, and is persistent 
	//compared to wireless logging
	TELEMETRY.setMode(Telemetry::Mode::SDCard, "run_" + std::to_string(runId) + ".ndjson");

	TELEMETRY.setEnabled(Telemetry::Channel::Pose, true);
	TELEMETRY.setEnabled(Telemetry::Channel::WallLocalization, true);
	TELEMETRY.setEnabled(Telemetry::Channel::Lift, false);
	TELEMETRY.setEnabled(Telemetry::Channel::Tuning, false);
	TELEMETRY.setEnabled(Telemetry::Channel::Debug, true);

	vert.odom_sensor == nullptr ? 0: vertRotation.set_position(0);
	horiz.odom_sensor == nullptr ? 0 : horizRotation.set_position(0);
	

	// Blocks ~2s while the IMU's gyro/accel finish their startup calibration,
	// waits until sensor ready basically before feeding ODOM calculations etc.
	//picked this tip up from Lemlib's library.
	chassis.calibrateIMU();


	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	//set motion params for KAV model
#ifdef ROBOT_MAIN
	chassis.set_speeds_lateral(Speed::SLOW, {40.0, 0.0, 65.0}); // cruise_vel, final_vel, accel
	chassis.set_speeds_lateral(Speed::NORMAL, {60.0, 0.0, 90.0}); // cruise_vel, final_vel, accel. was 75: every leg a triangle, peak 5-7V of 12 (run_110-121)
	chassis.set_speeds_lateral(Speed::FAST, {80.0, 0.0, 85.0}); // cruise_vel, final_vel, accel

	chassis.angular_kS = 700; //stall kick, run_57 stuck at 0.45deg with 400 kick + 200 P. Was 1910 (old drivetrain)
	chassis.lateral_kS = 390; //ff_lateral kS, residual push used by LateralPIDTune + tank_motion_profile
#else
	chassis.set_speeds_lateral(Speed::SLOW, {50.0, 0.0, 25.0});
	//was 60/120: FF needed ~11.7V at the top of the ramp (speed_test_14). 55/150 fits under ~11.3V
	chassis.set_speeds_lateral(Speed::NORMAL, {55.0, 0.0, 150.0});
	chassis.set_speeds_lateral(Speed::FAST, {70.0, 0.0, 120.0});

	chassis.angular_kS = 1071;
#endif
	logRobotConfig();

	//drive command
	CommandScheduler::registerSubsystem(&chassis, &arcadeDrive);
	CommandScheduler::registerSubsystem(&back_wall_sensor, nullptr);
	CommandScheduler::registerSubsystem(&front_wall_sensor, nullptr);
	CommandScheduler::registerSubsystem(&left_wall_sensor, nullptr);
	CommandScheduler::registerSubsystem(&right_wall_sensor, nullptr);
	if(wall_enabled){
	wallLocalization.schedule();
	}
#ifdef ROBOT_MAIN

#endif
#ifdef HAS_LIFT
	CommandScheduler::registerSubsystem(&lift, &liftTeleop);
	CommandScheduler::registerSubsystem(&claw, &clawTeleop);
#endif

	//runs CommandScheduler::run() every 10ms in its own task from here on (autonomous/opcontrol no longer call it)
	CommandScheduler::start();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


void autonomous() {
	//no teleop commands grabbing subsystems between auton steps, and clear out anything running from disabled
	CommandScheduler::setDefaultsEnabled(false);
	CommandScheduler::cancelAll();
	TELEMETRY.send(Telemetry::Channel::Debug, std::format(
		"{{\"t\": {}, \"event\": \"auton_start\"}}\n", pros::millis()));

	//AUTON SELECT: 0 = none (falls through to the test code below), 1 = RedHard, 2 = BlueHard, 3 = skills, 4 = skills_pt_3 (test), 5 = skills_optimized_test (speed experiment)
	constexpr int AUTON_SELECT = 3;
	if(AUTON_SELECT == 3){
#ifdef HAS_LIFT
		skills(&chassis, &wallLocalization, &claw, &lift); //full skills (part 1 + matchloader cycles), seeds its own start pose
#endif
		printf("END\n"); //tells auto_record.py to stop the camera recording
		return;
	}
	if(AUTON_SELECT == 4){
#ifdef HAS_LIFT
		skills_pt_3(&chassis, &wallLocalization, &claw, &lift); //skills part 3 on its own, start at the matchloader facing -X
#endif
		printf("END\n");
		return;
	}
	if(AUTON_SELECT == 5){
#ifdef HAS_LIFT
		skills_optimized_test(&chassis, &wallLocalization, &claw, &lift); //speed-first copy of skills, same start pose
#endif
		printf("END\n");
		return;
	}
	if(AUTON_SELECT != 0){
		//seed the start pose from the wall sensors, RedHard/BlueHard use their nominal start pose if a sensor misses
		bool seeded = (AUTON_SELECT == 1) ? wallLocalization.set_initial_pose(90, WallLocalization::Quadrant::NegXNegY)
		                                  : wallLocalization.set_initial_pose(270, WallLocalization::Quadrant::PosXPosY);
		Sequence* hardAuton = (AUTON_SELECT == 1) ? RedHard(&chassis, !seeded) : BlueHard(&chassis, !seeded);
		await(hardAuton); //blocks until the auton finishes, so END below really means it's done
		printf("END\n"); //tells auto_record.py to stop the camera recording
		return;
	}

#ifdef HAS_LIFT
	// lift.moveTo(LiftPos::SECOND_CONE)->schedule();
	claw.moveTo(-100)->schedule();
#else
	// wallLocalization.set_initial_pose(90, WallLocalization::Quadrant::PosXNegY); //reset pose once at beginning
	//nothing here since test bot no have any formal auton
//20 in y , 22.5 in x

	//TRACKING WHEEL OFFSET TEST: slow in-place spin(3 turns CCW, ~2.5V) for data analysis only.
	//Pose telemetry logs heading + vert + horiz, so offset = wheel distance / heading change.
	// (new RawTurn(&chassis, 1080, 2500, 14000))->schedule();

	//AUTON SEQUENCE TEST 1
	// (new Sequence({
	// 	chassis.moveToPoint(36.178, -16.616, 3.4),
	// 	chassis.moveToPoint(-0.807, -49.322, 3.4),
	// 	chassis.moveToPoint(-9.007, -46.877, 0, true)
	// }))->schedule();

	// AUTON SEQUENCE TEST 2
	// (new Sequence({
	// 	chassis.swing(180, DriveSide::LEFT, false, 127, 5), //early exit range was 10 before, as of 4:53PM im changing it to be higher for testing.
	// 	chassis.moveToPoint(-12.007, -46.877, 0)
	// }))->schedule();
	auton_1(&chassis, &wallLocalization);

#endif
	printf("END\n"); //tells auto_record.py to stop the camera recording

}


 //MAX SPEED TURNING IS 10.6-7 RAD/SEC!
 //MAX LATERAL IS 90in/sec
void opcontrol() {
	// Driver control coasts to a stop instead of braking - initialize() sets
	// BRAKE for autonomous accuracy, but that same setting was carrying over
	// into teleop and making the drivetrain grab when the sticks are released.
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	//stop whatever auton left running and hand subsystems back to the teleop commands
	CommandScheduler::cancelAll();
	CommandScheduler::setDefaultsEnabled(true);

	//ALL OF THE BELOW ARE SIMPLY CONFIG/TUNING TESTS:

	// {
	

	// chassis.setPose(0,0,0); //TODO remove this later. Temporarily here for testing.

	// One combined test (slow ramp + hard bursts), one joint kS/kV/kA fit -
	// see Commands/Tuning/DriveCharacterize. Replaces the old two-pass
	// quasistatic-then-residual-kA workflow.
	// DriveCharacterize kav(&chassis);
	// CommandScheduler::schedule(&kav);

	// wheel spin test: prop the robot up so the wheels are off the ground. Spins both sides at 3V for 10s,
	// count how many times a taped wheel goes around, then compare to MotL/MotR on the screen
	// RawDrive spinTest(&chassis, 9999, false, 3000, 10000);
	// spinTest.schedule();

	// CameraCharacterize cameraKav(&chassis);
	// CommandScheduler::schedule(&cameraKav);

	// AngularCharacterize ang(&chassis);
	// CommandScheduler::schedule(&ang);

	// FeedForwardTest ff_test(&chassis);
	// CommandScheduler::schedule(&ff_test);


	//Tuning channel is only turned on here, logs go to the SD run file
	// TELEMETRY.setEnabled(Telemetry::Channel::Tuning, true);
	// AngularPIDTune angularTune(&chassis, 150.0, 2500, 9000); // 150° step, 2.5s window, 9V cap (run_64/66 hopped at 12V)
	// angularTune.schedule();
	//LateralPIDTune lateralTune(&chassis, 36, 10000);
	//pure PID step (no FF), 36in, 4s window
	// LateralPIDTune lateralTune(&chassis, 36, 4000);
	// lateralTune.schedule();

	// Step-response test for cascade_lift_pid (declared near the other
	// controllers above, currently 0/0/0 - tune it here). targetDeg is a raw
	// lift encoder angle, same units as lift.moveTo(). Prints ~30hz JSON
	// (t, targetDeg, posDeg, errorDeg, mV) over TELEMETRY - see LiftPIDTune.
	// LiftPIDTune liftPidTune(&lift, &cascade_lift_pid, -2300, 3000);
	// liftPidTune.schedule();


	// LateralMotionDiagnostic motionDiag(&chassis, 70, /*usePID=*/true);
	// motionDiag.schedule();



	// UP/DOWN = +-1deg, LEFT/RIGHT = +-10deg dial in a rotate() target on the
	// controller screen, Y runs it - see Commands/Tuning/RotateDialTest.
	// RotateDialTest rotateDialTest(&chassis, &controller);
	// rotateDialTest.schedule();
	// MoveToPointDialTest mp(&chassis, &controller);
	// mp.schedule();

	// chassis.setPose(0, 0, 0);
	// odom_accuracy_test(&chassis, 24, 4000, 3000)->schedule();

	// }

	int counter = 0;

	uint32_t now = pros::millis();
	//scheduler runs in its own task now (see CommandScheduler::start()), this loop just has to stay alive
	//so the tuning commands above (stack variables) don't go out of scope
	while(true){
		counter++;

		if(counter == 10){
		//fixed 10hz stream of every wall sensor (raw, unfiltered, no gates) + odom pose, to compare against tape-measured position
		Pose p = chassis.gpos();
		float thetaDeg = radToDeg(p.theta);
		for(WallSensor* s : {&back_wall_sensor, &left_wall_sensor, &front_wall_sensor, &right_wall_sensor}){
			double wallDist = wallLocalization.get_dist_from_wall(s->side, thetaDeg);
			TELEMETRY.send(Telemetry::Channel::WallLocalization, std::format(
				"{{\"t\": {}, \"event\": \"wall_stream\", \"side\": \"{}\", \"dist\": {:.3f}, \"distFromWall\": {:.3f}, "
				"\"confidence\": {}, \"objectSize\": {}, \"x\": {:.3f}, \"y\": {:.3f}, \"theta\": {:.2f}}}\n",
				pros::millis(), (int)s->side, s->getDist(), wallDist, s->getConfidence(), s->getObjectSize(),
				p.x, p.y, thetaDeg));
		}
		// double theta = radToDeg(chassis.gpos().theta);
        // double d = wallLocalization.get_dist_from_wall(WallSensor::Side::RIGHT, radToDeg(chassis.gpos().theta));
        // TELEMETRY.debugWireless(std::format("theta: {:.1f}  dist_from_wall: {:.2f}", theta, d));

        // std::int32_t objSize = test_wall_sensor.sensor->get_object_size();
        // std::int32_t confidence = test_wall_sensor.sensor->get_confidence();
        // TELEMETRY.debugWireless(std::format("dist sensor - object size: {}  confidence: {}", objSize, confidence));

		counter=0;
		}
		// if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) && !angularTune.scheduled()){
		// 	angularTune.schedule();
		// }

		pros::delay(10); //yield, a bare busy loop starves lower priority tasks
	}
}
