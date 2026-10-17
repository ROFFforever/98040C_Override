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
#include "TestAutons/OdomCameraTests.h"
#include <format>
#include "Subsystems/piston.h"
#include "Commands/Tuning/AngularCharacterize.h"
#include "Commands/TeleopCommands/PistonTeleopCommand.h"
#include "Subsystems/WallSensor.h"
#include "Commands/Localization/WallLocalization.h"


#include "Commands/Tuning/AngularPIDTune.h"   // add near your other Commands includes
#include "Commands/Tuning/LateralPIDTune.h"
#include "Commands/Tuning/RotateDialTest.h"
#include "Commands/Tuning/MoveToPointDialTest.h"
#include "Commands/Tuning/LateralMotionDiagnostic.h"

bool wall_enabled=true;

//which odom/camera test autonomous() runs (TestAutons/OdomCameraTests), 0 = off.
//Picked with the brain screen's left/right buttons so it can change without re-uploading.
int odomTestId = 0;
void showOdomTest(){
	pros::lcd::set_text(2, odomTestId == 0 ? std::string("Odom test: off  (< > to pick)")
	                                       : std::format("Odom test: {}  (< > to pick)", odomTestId));
}

#ifdef ROBOT_MAIN
pros::MotorGroup leftMotors({11,5,4});   // port numbers; negative = reversed
pros::MotorGroup rightMotors({12,-7,6});
pros::Imu imu(4);
pros::Rotation vertRotation(6); //reverse angle
pros::Rotation horizRotation(3); //reverse angle
odom_wheel vert(&vertRotation, -0.875, 2.125);
odom_wheel horiz(nullptr, 0.75, 2.125);

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


#ifdef HAS_LIFT
pros::Motor lift_1(1);
//TODO TUNE: placeholder gains. Error is in degrees, output is mV. Add &lift_rotation as the 3rd Lift arg once the rotation sensor is installed
PID lift_pid(1200, 0, 0, 0, 0);
Lift lift({{&lift_1, false}}, &lift_pid);
pros::Motor claw_1(8);
Claw claw({{&claw_1, true}}); //inverted
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
PID angular_pid(15.2 * 1000.0,0,117 * 1000,0,0);


// //
// {"kS": 0.671754342335145, "kV": 0.13186696277648247, "kA": -0.0006958734869371364, "r2": 0.803989358366047, "rmse": 2.7154202310992996, "n": 1216, "vRange": [-123.13297872820009, 193.5721178570053], "aRange": [-19357.211785700532, 19357.211785700532]}
velocity_feed_forward ff_lateral(0.13186696277648247 * 1000,  // kV
                          0, // kA is 0 because it doens't pull much weight
                          0.671754342335145 * 1000);  // kS

velocity_feed_forward ff_angular(0.8995599372169607*1000,  // kV
0.089*1000, // kA
1.071431989714771 * 1000);  // kS

// Test robot currently has no intake, claw piston, or lift installed - just drivetrain.
#endif

pros::Controller controller(pros::E_CONTROLLER_MASTER);

#ifdef HAS_LIFT
LiftTeleopCommand liftTeleop(&lift, &controller, pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_R1);
ClawTeleopCommand clawTeleop(&claw, &controller, pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2);
#endif
#ifdef ROBOT_MAIN

#endif


//vertOffset was 8.625, then 7.2. 7.2 was measured from the camera's point, which sat ~0.8in behind the odom
//tracking point (tag profile fixed 10/16). Measured from the odom point it's 7.7-7.8
WallSensor back_wall_sensor(14, -2.75f, 7.75f,  WallSensor::Side::BACK);

//vertOffset=0.75  horizOffset=7.26
WallSensor left_wall_sensor(9, 2.25f, 6.875f,  WallSensor::Side::LEFT);

// vertOffset=7.30  horizOffset=1.80
WallSensor front_wall_sensor(17, 1.80f, 8.1f,  WallSensor::Side::FRONT);


WallSensor right_wall_sensor(4, -2.625f, 6.50f,  WallSensor::Side::RIGHT);


drivetrain chassis(&leftMotors, &rightMotors, &imu, Units::WHEEL_325, 360, &vert, &horiz, &angular_pid, &ff_lateral, &ff_angular, &residual_lateral_PID); // 450 = wheel's actual output rpm after gearing
WallLocalization wallLocalization({&back_wall_sensor, &left_wall_sensor, &front_wall_sensor, &right_wall_sensor}, &chassis);
ArcadeDriveCommand arcadeDrive(&chassis, &controller); // drivetrain's default teleop command

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
	lift.holdPosition();
	lift.setDebugStream(true); //set true to stream lift position over pros terminal every 0.5s
#endif

	int runId = nextRunId();
	//a message for the recording script to start(provides run ID)
	printf("RUN_ID:%d\n", runId);
	fflush(stdout);

	pros::lcd::initialize();
	pros::lcd::register_btn0_cb([]{ odomTestId = (odomTestId + ODOM_TEST_COUNT) % (ODOM_TEST_COUNT + 1); showOdomTest(); });
	pros::lcd::register_btn2_cb([]{ odomTestId = (odomTestId + 1) % (ODOM_TEST_COUNT + 1); showOdomTest(); });
	showOdomTest();

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
	
	pros::lcd::set_text(1, "Hello PROS User!");

	// Blocks ~2s while the IMU's gyro/accel finish their startup calibration,
	// waits until sensor ready basically before feeding ODOM calculations etc.
	//picked this tip up from Lemlib's library.
	chassis.calibrateIMU();


	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	//set motion params for KAV model
#ifdef ROBOT_MAIN
	chassis.set_speeds_lateral(Speed::SLOW, {40.0, 0.0, 65.0}); // cruise_vel, final_vel, accel
	chassis.set_speeds_lateral(Speed::NORMAL, {60.0, 0.0, 75.0}); // cruise_vel, final_vel, accel
	chassis.set_speeds_lateral(Speed::FAST, {80.0, 0.0, 85.0}); // cruise_vel, final_vel, accel

	chassis.angular_kS = 1910;
#else
	chassis.set_speeds_lateral(Speed::SLOW, {50.0, 0.0, 25.0});
	chassis.set_speeds_lateral(Speed::NORMAL, {60.0, 0.0, 120.0});
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
		"{{\"t\": {}, \"event\": \"auton_start\", \"odomTestId\": {}}}\n", pros::millis(), odomTestId));

#ifdef HAS_LIFT
	lift.moveTo(LiftPos::SECOND_CONE)->schedule();
#else
	if(odomTestId != 0){
		if(odom_test_init_pose(&wallLocalization, &chassis)){
			odom_camera_test(odomTestId, &chassis)->schedule();
		}
		return;
	}

	wallLocalization.set_initial_pose(90, WallLocalization::Quadrant::PosXNegY); //reset pose once at beginning
	//nothing here since test bot no have any formal auton
//20 in y , 22.5 in x

	//TRACKING WHEEL OFFSET TEST: slow in-place spin(3 turns CCW, ~2.5V) for data analysis only.
	//Pose telemetry logs heading + vert + horiz, so offset = wheel distance / heading change.
	(new RawTurn(&chassis, 1080, 2500, 14000))->schedule();

	//AUTON SEQUENCE TEST 1
	// (new Sequence({
	// 	chassis.moveToPoint(36.178, -16.616, 3.4),
	// 	chassis.moveToPoint(-0.807, -49.322, 3.4),
	// 	chassis.moveToPoint(-9.007, -46.877, 0, true)
	// }))->schedule();

	// AUTON SEQUENCE TEST 2
	// (new Sequence({
	// 	chassis.swing(180, DriveSide::LEFT, false, 5), //early exit range was 10 before, as of 4:53PM im changing it to be higher for testing.
	// 	chassis.moveToPoint(-12.007, -46.877, 0)
	// }))->schedule();
	
#endif
	
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

	// CameraCharacterize cameraKav(&chassis);
	// CommandScheduler::schedule(&cameraKav);

	// AngularCharacterize ang(&chassis);
	// CommandScheduler::schedule(&ang);

	// FeedForwardTest ff_test(&chassis);
	// CommandScheduler::schedule(&ff_test);


	// AngularPIDTune angularTune(&chassis, 90.0, 2500); // 45° step, 2.5s window
	// angularTune.schedule();
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

	}
}
