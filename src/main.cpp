#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp"


bool firstPos = true;
bool secondPos = false;
bool thirdPos = false;


// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER); //! SIGMA CONTROLLER 🤖
lemlib::ExpoDriveCurve driveCurve(3, 10, 1.019);

// Motor Groups
pros::MotorGroup left_mg({-19, -18, -17}, pros::MotorGearset::blue);    
pros::MotorGroup right_mg({12, 13, 14}, pros::MotorGearset::blue); 

// Single Motors
pros::Motor flexwheel(20, pros::MotorGearset::green);
pros::Motor chain(11, pros::MotorGearset::blue);
pros::Motor ladyBrown(8, pros::MotorGearset::green);

// Solenoids
pros::adi::Pneumatics mogo('A', false);
pros::adi::Pneumatics doinker('B', false); //? IT DOINKS!!!!

// Drivetrain
lemlib::Drivetrain drivetrain(
	&left_mg, // left motor group
    &right_mg, // right motor group
    11.25, // 11.25 inch track width
	lemlib::Omniwheel::NEW_325, // using new 3.25" omnis //! Check wheel size
    450, // drivetrain rpm is 450
    2 // horizontal drift is 2 (for now)
);


// sensors
pros::Imu imu(15); // degrees/turning
pros::Distance distance_sensor(9);  // Assuming it's connected to port 1



// pros::Rotation xOdom(6); // x pos place on left or right sides at base of dt
pros::Rotation yOdom(-16); // y pos place on front or back of dt
pros::Rotation ladyOdom(9); // y pos place on front or back of dt

//! Need to adjust offsets -> Distance from Center of wheel to center of tracking wheel(left -> neg; right -> pos)

// lemlib::TrackingWheel horizontal_tracking_wheel(&xOdom, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&yOdom, lemlib::Omniwheel::NEW_2, -2.5); //! offset
// lemlib::TrackingWheel lady_tracking_wheel(&yOdom, lemlib::Omniwheel::NEW_275, -2.5);

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);


// PID(After you tune, everything should work as expect, if not look at the printscreens to see if the robot thinks its getting to such positions, if so, then its likely a PID tuning issue, will take sometime but you have time)
// Do default, set 0, slowly adjust each factor, 
// at the end, should have smooth motions

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
lemlib::Chassis chassis(
    drivetrain, // drivetrain settings
    lateral_controller, // lateral PID settings
    angular_controller, // angular PID settings
    sensors, // odometry sensors
    &driveCurve,
    &driveCurve
);

//? Random code ->

void on_center_button() {
	// static bool pressed = false;
    // pressed = !pressed;
    // if (pressed) {
    //     if (selectedSide == BLUE_RIGHT) {
    //         selectedSide = RED_LEFT;
    //         pros::lcd::set_text(2, "Selected: Red Left");
    //     } else {
    //         selectedSide = BLUE_RIGHT;
    //         pros::lcd::set_text(2, "Selected: Blue Right");
    //     }
    // }
}

void initialize() {
	pros::lcd::initialize();
    // calibrate sensors
	imu.set_heading(0); // Set IMU heading to 0 for reference
    pros::delay(100); // Optional delay for IMU calibration

    // xOdom.reset_position(); // Reset X odom sensor
    yOdom.reset_position(); // Reset Y odom sensor
    ladyOdom.reset_position();
	pros::delay(100); // Optional delay for IMU calibration

    chassis.calibrate(); // Calibrate the chassis to reset odometry
	pros::delay(500);

	pros::lcd::print(1, "Calibration Complete 🍎 Sigma");


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
    // yOdom.reset_position();
    // chassis.calibrate(); // Calibrate the chassis to reset odometry

	imu.set_heading(0);
	chassis.setPose(0,0,180);
    // Below code should move robot to point 20" in a straight line, with the most amount of time it can move for is 10000 ms(complete action in that time)
    chassis.moveToPoint(0, 33, 2000, {.forwards=false, .maxSpeed=85});
    pros::delay(800);
    mogo.extend();
    chain.move(200);
    pros::delay(400);
    flexwheel.move(200);
    chassis.turnToHeading(280,700);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 19, 3000, {.forwards=true, .maxSpeed=85});
    chassis.waitUntilDone();
    chassis.turnToHeading(80, 600);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 9, 1000, {.forwards=true, .maxSpeed=85});
    chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();    
    chassis.moveToPoint(0, -5, 600, {.forwards=false, .maxSpeed=85});
    chassis.turnToHeading(330, 500);    
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 16, 1300, {.forwards=true, .maxSpeed=85});
    pros::delay(2500);
    chassis.moveToPoint(0, -18, 1300, {.forwards=false, .maxSpeed=85});
    chassis.turnToHeading(250, 500);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 28, 3000, {.forwards=true, .maxSpeed=65});
} 

void opcontrol() {
	while (true) {

        // movement
		int leftY = master.get_analog(ANALOG_LEFT_Y);
		int rightY = master.get_analog(ANALOG_RIGHT_Y);
		left_mg.move(leftY);
		right_mg.move(rightY);

        int sensor_value = distance_sensor.get_distance();


		pros::delay(20); // Run for 20 ms then update

        //! Mogo Clamp
        if(master.get_digital(DIGITAL_B)){
            mogo.extend();
        } else if(master.get_digital(DIGITAL_DOWN)){
            mogo.retract();
        }

        if(master.get_digital(DIGITAL_RIGHT)){
            if(sensor_value < 10.0){
                pros::delay(10);
                chain.move(-400);
                flexwheel.move(-400);
                pros::delay(800);  // Delay for 500ms (duration of the outtake)
                chain.move(0);
                flexwheel.move(0);
            }
        }

        //! Doinker;
        if(master.get_digital(DIGITAL_L1)){
            doinker.set_value(true);
        } else {
            doinker.set_value(false);
        }


        //! Intake
        if(master.get_digital(DIGITAL_R1)){
            chain.move(400);
            flexwheel.move(400);
        } else if(master.get_digital(DIGITAL_R2)){
            chain.move(-400);
            flexwheel.move(-400);
        } else{
            chain.move(0);
            flexwheel.move(0);
        }

        //! Lady Brown
        if(master.get_digital(DIGITAL_L2)) {
            if(firstPos == true && secondPos == false && thirdPos == false){
                    ladyBrown.move_absolute(390, 200);
                    firstPos = false;
                    secondPos = true;
                    thirdPos = false;
            } 
            else if(firstPos == false && secondPos == true && thirdPos == false){
                    ladyBrown.move_absolute(1800, 200);
                    firstPos = false;
                    secondPos = false;
                    thirdPos = true;
            }
            else if(firstPos == false && secondPos == false && thirdPos == true){
                    ladyBrown.move_absolute(0, 250);
                    firstPos = true;
                    secondPos = false;
                    thirdPos = false;
            }   
        }
        

	}
}