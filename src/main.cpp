#include "main.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp"


/* TODO: 
1) make all basic funtionalities; 
2) setup odom sensors/test via print to brain screen;
3) PID tune; 
4) experiment with tank drive; 
5) auton selector; 
*/


// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER); //! SIGMA CONTROLLER 🤖
lemlib::ExpoDriveCurve driveCurve(5, 12, 1.132);

// Motor Groups
pros::MotorGroup left_mg({-1, -2, -3}, pros::MotorGearset::blue);    
pros::MotorGroup right_mg({5, 11, 8}, pros::MotorGearset::blue); 

// Single Motors
pros::Motor flexwheel(18);
pros::Motor chain(15);
pros::Motor ladyBrown(21);

// Solenoids
pros::adi::Pneumatics mogo('A', false);
pros::adi::Pneumatics doinker('B', false); //? IT DOINKS!!!!

// Drivetrain
lemlib::Drivetrain drivetrain(
	&left_mg, // left motor group
    &right_mg, // right motor group
    11.25, // 11.25 inch track width
	lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    450, // drivetrain rpm is 450
    2 // horizontal drift is 2 (for now)
);


// sensors
pros::Imu imu(10);

//? Odom Setup

pros::Rotation xOdom(6);
pros::Rotation yOdom(7);

//! Need to adjust offsets -> Distance from Center of wheel to center of tracking wheel(left - neg; right - pos)

lemlib::TrackingWheel horizontal_tracking_wheel(&xOdom, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&yOdom, lemlib::Omniwheel::NEW_275, -2.5);

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);


// PID

// TODO: Tune PID again with new sensors

lemlib::ControllerSettings lateral_controller(
	10, // proportional gain (kP)
    0, // integral gain (kI)
    3, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
	100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);


lemlib::ControllerSettings angular_controller(
	2, // proportional gain (kP)
	0, // integral gain (kI)
    10, // derivative gain (kD)
	3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


//? Random code ->

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();
    // calibrate sensors
	imu.set_heading(0); // Set IMU heading to 0 for reference
    pros::delay(100); // Optional delay for IMU calibration

    xOdom.reset_position(); // Reset X odom sensor
    yOdom.reset_position(); // Reset Y odom sensor
	pros::delay(100); // Optional delay for IMU calibration

    chassis.calibrate(); // Calibrate the chassis to reset odometry
	pros::delay(200);

	pros::lcd::print(1, "Calibration Complete 🍎 Skib");


	pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::delay(20);
        }
    });
}

void disabled() {} // NOT NEEDED!!!

void competition_initialize() {} // NOT NEEDED!

void autonomous() {
	// Setting everything to 0,0,0
	imu.set_heading(0);
	chassis.setPose(0,0,0);


	// Below code should move robot to point 0,20 in a straight line, with the most amount of time it can move for is 10000 ms
	chassis.moveToPose(0, 20, 0, 10000);

} 

void opcontrol() {
	while (true) {
		int leftY = master.get_analog(ANALOG_LEFT_Y);
		int rightY = master.get_analog(ANALOG_RIGHT_Y);
		left_mg.move(leftY);
		right_mg.move(rightY);
		pros::delay(20); // Run for 20 ms then update
	}
}