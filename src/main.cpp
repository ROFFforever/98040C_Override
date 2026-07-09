#include "main.h"
#include "Units.h"
#include "Subsystems/drivetrain.h"
#include "Telemetry/telemetry.h"
#include "CommandScheduler/commandScheduler.h"
#include "Commands/ArcadeDriveCommand.h"
#include "Commands/IntakeTeleopCommand.h"
#include "Subsystems/intake.h"

pros::MotorGroup leftMotors({13, -12, -11});   // port numbers; negative = reversed
pros::MotorGroup rightMotors({-18, 19, 20});
pros::Motor intake_motor_1(1);
pros::Imu imu(7);                  // port 7


pros::Controller controller(pros::E_CONTROLLER_MASTER);
drivetrain chassis(&leftMotors, &rightMotors, &imu, 450); // 450 = wheel's actual output rpm after gearing
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
	while(true){
		
		// CommandScheduler::run() replaces the manual calls above: it calls
		// periodic() on every registered Subsystem (myDrive included), and
		// runs execute() on whichever Command currently owns each Subsystem
		// (arcadeDrive, until something else schedules over it).
		CommandScheduler::run();
		pros::delay(16); //60hz
	}
}