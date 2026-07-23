#include "main.h"
#include "Units.h"
#include "Subsystems/drivetrain.h"
#include "Telemetry/telemetry.h"
#include "CommandScheduler/commandScheduler.h"
#include "Commands/ArcadeDriveCommand.h"
#include "Commands/IntakeTeleopCommand.h"
#include "Subsystems/intake.h"

pros::MotorGroup leftMotors({18, 20});   // port numbers; negative = reversed
pros::MotorGroup rightMotors({-11, -12});
pros::Motor intake_motor_1(1);
pros::Imu imu(10);                  // port 7


pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Rotation vertRotation(1);           
odom_wheel vert(&vertRotation, 0, 2.0);
odom_wheel horiz(nullptr, 0, 0);
drivetrain chassis(&leftMotors, &rightMotors, &imu, Units::WHEEL_325, 450, &vert, &horiz); // 450 = wheel's actual output rpm after gearing
intake intake_motors({&intake_motor_1}, false);
ArcadeDriveCommand arcadeDrive(&chassis, &controller); // drivetrain's default teleop command
IntakeTeleopCommand intakeTeleop(&intake_motors, &controller, pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_L1);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	// Blocks ~2s while the IMU's gyro/accel finish their startup calibration,
	// so nothing downstream (odom, telemetry) reads garbage headings before
	// the sensor is actually ready.
	chassis.calibrateIMU();

	// From now on, whenever nothing else has claimed myDrive, the scheduler
	// runs arcadeDrive on it - this is what makes teleop driving "just work"
	// once CommandScheduler::run() is looping in opcontrol().
	CommandScheduler::registerSubsystem(&chassis, &arcadeDrive);
	CommandScheduler::registerSubsystem(&intake_motors, &intakeTeleop);
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

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	Timer time(10);
	chassis.ticks = &time;
	while(true){
		CommandScheduler::run();
		pros::delay(16); //60hz
	}
}