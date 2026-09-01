#include "main.h"
#include "Commands/TeleopCommands/LiftTeleopCommand.h"
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
#include "Commands/Tuning/DriveCharacterize.h"
#include "Telemetry/telemetry.h"
#include "Commands/Tuning/FeedForwardTest.h"
#include "Commands/Rotate.h"
#include "Subsystems/Lift.h"
#include "Commands/LiftMoveToCommand.h"
#include "Autons/SoloAWP.h"
#include "Subsystems/piston.h"
#include "Commands/Tuning/AngularCharacterize.h"
#include "Commands/TeleopCommands/PistonTeleopCommand.h"


#include "Commands/Tuning/AngularPIDTune.h"   // add near your other Commands includes
#include "Commands/Tuning/LateralPIDTune.h"
#include "Commands/Tuning/LiftPIDTune.h"
#include "Commands/Tuning/RotateDialTest.h"
#include "Commands/Tuning/MoveToPointDialTest.h"
#include "Commands/Tuning/LateralMotionDiagnostic.h"

//TEST ROBOT
// pros::MotorGroup leftMotors({18, 20});   // port numbers; negative = reversed
// pros::MotorGroup rightMotors({-11, -12});
// pros::Imu imu(10);         
// pros::Rotation vertRotation(-16); //reverse angle   
// pros::Rotation horizRotation(15); //reverse angle  
// odom_wheel vert(&vertRotation, 1.25, 2.125);
// odom_wheel horiz(&horizRotation, 0.75, 2.125);  
//TEST ROBOT CONTROLLERS:
//controllers
// PID residual_lateral_PID(2.4 * 1000,0,70*100,0,0);
// PID angular_pid(20 * 1000.0,0,120 * 1000,0,0);

// velocity_feed_forward ff_lateral(0.14954251997144965 * 1000,  // kV
//                           0, // kA is 0 because it doens't pull much weight
//                           0.9570910152819988 * 1000);  // kS

// velocity_feed_forward ff_angular(0.8995599372169607*1000,  // kV
// 0.089*1000, // kA
// 1.071431989714771 * 1000);  // kS  


//NEW ROBOT STUFF:
pros::Motor lift_1(5);
pros::Motor lift_2(7);
pros::Rotation vertRotation(6); //reverse angle   
pros::MotorGroup leftMotors({17, 20});   // port numbers; negative = reversed
pros::MotorGroup rightMotors({-18, -19});
pros::Motor intake_motor_1(15);
pros::Motor intake_motor_2(16);
odom_wheel vert(&vertRotation, -0.875, 2.125);
odom_wheel horiz(nullptr, 0.75, 2.125);
pros::Imu imu(4);
pros::Rotation horizRotation(3); //reverse angle  
pros::adi::DigitalOut piston_1('a');
Intake intake_motors({{&intake_motor_1, false},{&intake_motor_2, false}});
PID cascade_lift_pid(140,0,0,0,0);
Lift lift({{&lift_1, false}, {&lift_2, true}}, &cascade_lift_pid);
piston claw_piston(piston_1);
pros::Controller controller(pros::E_CONTROLLER_MASTER);  
IntakeTeleopCommand intakeTeleop(&intake_motors, &controller, pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_L1);
LiftTeleopCommand liftTeleop(&lift, &controller, pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_R1);
PistonTeleopCommand clawPistonTeleop(&claw_piston, &controller, pros::E_CONTROLLER_DIGITAL_A);





//controllers
PID residual_lateral_PID(0.95 * 1000,0,26*100,0,0);
//PID residual_lateral_PID(0 * 1000,0,0*100,0,0);
PID angular_pid(26 * 1000.0,0,1457 * 100,0,0);
velocity_feed_forward ff_lateral( 0.1384 * 1000,  // kV
                          0.024 * 1000, // kA is 0 because it doens't pull much weight
                          1.551 * 1000);  // kS

velocity_feed_forward ff_angular(0.931 *1000,  // kV
0.1025*1000, // kA
1.91 * 1000);  // kS


drivetrain chassis(&leftMotors, &rightMotors, &imu, Units::WHEEL_325, 360, &vert, &horiz, &angular_pid, &ff_lateral, &ff_angular, &residual_lateral_PID); // 450 = wheel's actual output rpm after gearing

ArcadeDriveCommand arcadeDrive(&chassis, &controller); // drivetrain's default teleop command
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

	// TELEMETRY defaults to Mode::Wireless. Once an SD card is in the robot,
	// switch to it like this (falls back to Wireless automatically if no
	// card is detected, so this is safe to leave in even without one):
	TELEMETRY.setMode(Telemetry::Mode::SDCard, "telemetry_log.txt");

	vert.odom_sensor == nullptr ? 0: vertRotation.set_position(0);
	horiz.odom_sensor == nullptr ? 0 : horizRotation.set_position(0);
	// Do NOT resetPosition() here - the lift has no limit switch/rotation sensor, so its
	// only position reference IS the motor's own encoder count, which already persists
	// across program restarts as long as the motor keeps power. Taring it on every
	// initialize() would throw that reference away and re-zero wherever the lift happens
	// to physically be sitting at the moment (which varies match to match).
	pros::lcd::set_text(1, "Hello PROS User!");

	// Blocks ~2s while the IMU's gyro/accel finish their startup calibration,
	// so nothing downstream (odom, telemetry) reads garbage headings before
	// the sensor is actually ready.
	chassis.calibrateIMU();

	// Coast is the PROS default; without this, cutting voltage to 0 at the end of a
	// motion lets momentum carry the robot further (coast-through overshoot).
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	// Conservative starting values for testing with no PID tuned yet.
	// Theoretical max from gearing (~61 in/s) and characterized ff_lateral (~74 in/s);
	// cruise_vel kept well under both so the motion doesn't outrun the feedforward model.
	chassis.set_speeds_lateral(Speed::SLOW, {40.0, 0.0, 65.0}); // cruise_vel, final_vel, accel
	chassis.set_speeds_lateral(Speed::NORMAL, {60.0, 0.0, 75.0}); // cruise_vel, final_vel, accel
	chassis.set_speeds_lateral(Speed::FAST, {80.0, 0.0, 85.0}); // cruise_vel, final_vel, accel

	chassis.set_speeds_angular(Speed::SLOW, {7, 0.0, 10}); // cruise_vel, final_vel, accel
	chassis.set_speeds_angular(Speed::NORMAL, {12, 0.0, 17.0}); // cruise_vel, final_vel, accel
	chassis.set_speeds_angular(Speed::FAST, {15, 0.0, 20.0}); // cruise_vel, final_vel, accel

	chassis.angular_kS = 1910;

	// From now on, whenever nothing else has claimed myDrive, the scheduler
	// runs arcadeDrive on it - this is what makes teleop driving "just work"
	// once CommandScheduler::run() is looping in opcontrol().
	CommandScheduler::registerSubsystem(&chassis, &arcadeDrive);
	CommandScheduler::registerSubsystem(&intake_motors, &intakeTeleop);
	CommandScheduler::registerSubsystem(&lift, &liftTeleop);
	CommandScheduler::registerSubsystem(&claw_piston, &clawPistonTeleop);
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

	//let down the toggle switcher
	intake_motor_2.move_voltage(-12000);
	pros::delay(450);
		intake_motor_2.move_voltage(0);

	(new Sequence({
		first_alliance_pin(&chassis, &claw_piston, &lift),
		// get_second_pin(&chassis, &claw_piston, &lift),
		go_back_toggle(&chassis, &claw_piston, &lift)
	}))->schedule();
	while(true){
    CommandScheduler::run();
    pros::delay(10);
	}
}


 //MAX SPEED TURNING IS 10.6-7 RAD/SEC!
 //MAX LATERAL IS 90in/sec
void opcontrol() {
	// Driver control coasts to a stop instead of braking - initialize() sets
	// BRAKE for autonomous accuracy, but that same setting was carrying over
	// into teleop and making the drivetrain grab when the sticks are released.
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	// chassis.setPose(0,0,0); //TODO remove this later. Temporarily here for testing.

	// One combined test (slow ramp + hard bursts), one joint kS/kV/kA fit -
	// see Commands/Tuning/DriveCharacterize. Replaces the old two-pass
	// quasistatic-then-residual-kA workflow.
	// DriveCharacterize kav(&chassis);
	// CommandScheduler::schedule(&kav);

	// AngularCharacterize ang(&chassis);
	// CommandScheduler::schedule(&ang);

	// FeedForwardTest ff_test(&chassis);
	// CommandScheduler::schedule(&ff_test);


	// AngularPIDTune angularTune(&chassis, 90.0, 2500); // 45° step, 2.5s window
	//LateralPIDTune lateralTune(&chassis, 36, 10000);

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

	while(true){
		CommandScheduler::run();

		// if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) && !angularTune.scheduled()){
		// 	angularTune.schedule();
		// }

		// Hold the lift at its physical home position and tap Y to re-zero it there -
		// its only position reference is the motor encoder, which drifts from that
		// home point over a session of testing/direction changes.
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			lift.resetPosition();
		}

		pros::delay(10); //100hz
	}
}