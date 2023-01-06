#include "main.h"
#include "pros/error.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <string>
#include <sys/errno.h>



// Define Ports for Motors
#define CHASSIS_L1_PORT 11
#define CHASSIS_L2_PORT 12
#define CHASSIS_L3_PORT 13
#define CHASSIS_R1_PORT 16
#define CHASSIS_R2_PORT 17
#define CHASSIS_R3_PORT 18
#define INTAKE_PORT 10
#define FLYWHEEL_A1_PORT 1
#define FLYWHEEL_A2_PORT 1
#define FLYWHEEL_PORT 1
#define ROLLER_PORT 1

// Define Ports for Sensors
#define ROLLER_SENSOR_PORT 1
#define TRACKING_SIDE_PORT 14
#define TRACKING_FORWARD_PORT 15
#define GYRO_PORT 19

// Define Ports for sensors and pistons on the Analog Ports
#define FLYWHEEL_ANGLE_PORT 1
#define INDEXER_PORT 2
#define ENDGAME_PORT 1


// The value of pi
static constexpr double pi = 3.1415926535897932;
// The value of pi divided by 2
static constexpr double pi2 = 1.5707963267948966;
// Converts inches to millimeters
static constexpr double inch_to_mm = 25.4;
// Converts millimeters to inches
static constexpr double mm_to_inch = 0.0393700787;
// Converts degrees to radians
static constexpr double degree_to_radian = 0.01745329252;
// Converts centidegrees to radians
static constexpr double centidegree_to_radian = .0001745329252;
// Converts radians to degrees
static constexpr double radian_to_degree = 57.2957795;
// Measurement in mm of 1 centidegree of the tracking wheels
static constexpr double track_wheel_size = 0.00609556241;
// Distance from back tracking wheel to center of robot
static constexpr double back_track_offset = 150;

// Varaiables to keep track of the Location of the Robot
double x_loc = 0, y_loc = 0, x_loc2 = 0, y_loc2 = 0;
// Variable to keep track of the Orientation of the Robot
double theta = 0;


// Define Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// Define Motors
pros::Motor chassis_r1(CHASSIS_R1_PORT);
pros::Motor chassis_r2(CHASSIS_R2_PORT);
pros::Motor chassis_r3(CHASSIS_R3_PORT);
pros::Motor chassis_l1(CHASSIS_L1_PORT, true);
pros::Motor chassis_l2(CHASSIS_L2_PORT, true);
pros::Motor chassis_l3(CHASSIS_L3_PORT, true);
pros::Motor intake(INTAKE_PORT, true);
pros::Motor flywheel(FLYWHEEL_PORT);
pros::Motor roller(ROLLER_PORT);
pros::Motor flywheel_a1(FLYWHEEL_A1_PORT);
pros::Motor flywheel_a2(FLYWHEEL_A2_PORT);
// Define Pistons
pros::ADIDigitalOut indexer(INDEXER_PORT);
// Define Sensors
pros::Optical roller_sensor(ROLLER_SENSOR_PORT);
pros::Rotation tracking_side(TRACKING_SIDE_PORT);
pros::Rotation tracking_forward(TRACKING_FORWARD_PORT);
pros::ADIPotentiometer flywheel_angle(FLYWHEEL_ANGLE_PORT);
pros::IMU gyro(GYRO_PORT);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
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
	pros::delay(3000);


	// Gets the position of the tracking wheels
	int track_side = tracking_side.get_position(), track_forward = tracking_forward.get_position();
	// Gets the rotation of the gyro sensor
	double track_theta = gyro.get_rotation();
	// Stores the previous iteration of tracking wheel positions
	int prev_track_side = track_side, prev_track_forward = track_forward;
	// Stores the previous gyro sensor value
	double prev_theta = track_theta;
	// Variables to store the target speed for each side of the chassis
	int leftSpeed = 0, rightSpeed = 0;
	// Flag to set the position of the indexing piston
	int indexing_flag = 0;
	// Sensor values for the movement of the tracking wheels.
	double mm_forward = 0, mm_side = 0;
	// The change in theta
	double delta_theta = 0;
	// The calculated distance from the center of the robot to the turning radius
	double radius = 0;
	// The calculated change in x and y position on the local coordinate system
	double x_local = 0, y_local = 0;
	// Flag to prevent division by 0 error in odometry determines if the change in theta is 0
	bool zero_theta = true;
	// Main Control Loop
	int iteration = 0;
	while (true) {
		
		// Get the sensor values for the tracking wheels and inertial sensor
		track_forward = tracking_forward.get_position();
		track_side = tracking_side.get_position();
		track_theta = gyro.get_rotation();



		// Converts sensor values to respective standardized units (radians and mm)
		mm_forward = (prev_track_forward - track_forward)*track_wheel_size;
		mm_side = (track_side - prev_track_side)*track_wheel_size;
		delta_theta = (track_theta - prev_theta)*centidegree_to_radian;

		
		// Determines if delta theta is zero to prevent division by 0 error
		zero_theta = true;//fabs(delta_theta) <= 0.001;

		/**
		 * Calculate the polar radius of the robots ending position relative to the local Polar
		 * Coordinate System where the origin is the center of rotation and theta=0 is the 
		 * starting angle.
		 *
		 * By summing the following:
		 *
		 * The average of the starting radius and the ending radius
		 * Half the difference of the starting radius and ending radius
		 * The distance from the back tracking wheel to the center of the robot
		 */
		if (zero_theta) {
			x_local = mm_forward;
			y_local = mm_side;
		} else {
			radius  = mm_side / delta_theta + mm_forward/2 + back_track_offset;
			x_local = sin(delta_theta) * radius - sin(delta_theta) * (radius - mm_forward);
			y_local = cos(delta_theta) * radius - cos(delta_theta) * (radius - mm_forward);
		}
		// radius  = zero_theta ? mm_side / delta_theta + mm_forward/2 + back_track_offset : 1;
		// x_local = zero_theta ? sin(delta_theta) * radius - sin(delta_theta) * (radius - mm_forward) : mm_forward;
		// y_local = zero_theta ? cos(delta_theta) * radius - cos(delta_theta) * (radius - mm_forward) : mm_side;
		
		// Converts the local movements to the global position
		x_loc += cos(theta*degree_to_radian)*x_local - sin(theta*degree_to_radian)*y_local;
		y_loc += sin(theta*degree_to_radian)*x_local + cos(theta*degree_to_radian)*y_local;
		theta = track_theta;

		//radius  = zero_theta ? 1 : mm_forward/delta_theta + mm_side/2; 
		//x_local = zero_theta ? mm_side : cos(delta_theta) * radius - cos(delta_theta) * (radius - mm_side);
		//y_local = zero_theta ? mm_forward : sin(delta_theta) * radius - sin(delta_theta) * (radius - mm_side);
	
		//pros::lcd::set_text(0, std::to_string());
		//pros::lcd::set_text(1, std::to_string());
		pros::lcd::set_text(2, std::to_string(x_loc));
		pros::lcd::set_text(3, std::to_string(y_loc));	
		pros::lcd::set_text(4, std::to_string(theta));
		//pros::lcd::set_text(5, std::to_string());
		//pros::lcd::set_text(6, std::to_string());
		// pros::lcd::set_text(7, std::to_string());

		if (isnanf(x_loc)) break;

		// Stores current sensor values to previous variables for use in next iteration
		prev_track_forward = track_forward;
		prev_track_side = track_side;
		prev_theta = track_theta;


		

		// Debugging for the Intake
		if (controller.get_digital(DIGITAL_R1)) {
			intake = 127;
		} else if (controller.get_digital(DIGITAL_R2)) {
			intake = 60;
		} else if (controller.get_digital(DIGITAL_L1)) {
			intake = -127;
		} else if (controller.get_digital(DIGITAL_L2)) {
			intake = -60;
		} else {
			intake = 0;
		}


		
		
		/**
		 * When A is pressed starts a timer of 5 iterations until piston retracts
		 * Piston expands if timer value is positive
		 * otherwise piston retracts
		 */
		if (controller.get_digital_new_press(DIGITAL_A)) 
			indexing_flag = 5;
		indexer.set_value(indexing_flag >= 0);
		indexing_flag -= 1;
		

		/** 
         * Arcade Controls 
         *
         * Commented out Arcade Controls
         * Easy to change based on user preference
         *
         * Sets the power of the chassis based on the left joysticks forward value
         * Increases and reduces power of left and right sides of the chassis based on the horizontal
         * value of the left joystick. Respectively turns chassis left and right based on value.
         */ 
		// int power = controller.get_analog(ANALOG_LEFT_Y); 
		// int turn = controller.get_analog(ANALOG_LEFT_X); 
		// int rightSpeed = power - turn; 
		// int leftSpeed = power + turn; 
 
		/** 
         * Base Tank Controls 
         *
         * Sets left side of Chassis to left joystick value
         * Sets right side of chassis to right joystick value
         * 
         * Joystick has a small deadzone to prevent accidental movements when 
         * the joysticks are not perfectly centered
         */ 
		leftSpeed = abs(controller.get_analog(ANALOG_LEFT_Y)) > 8 ? controller.get_analog(ANALOG_LEFT_Y) : 0; 
		rightSpeed = abs(controller.get_analog(ANALOG_RIGHT_Y)) > 8 ? controller.get_analog(ANALOG_RIGHT_Y) : 0; 

        // Set Chassis Motors to Target Speed
		chassis_l1 = leftSpeed;
		chassis_l2 = leftSpeed;
		chassis_l3 = leftSpeed;
		chassis_r1 = rightSpeed;
		chassis_r2 = rightSpeed;
		chassis_r3 = rightSpeed;

		
		

		//x_loc = original_x_loc + tracking_p.get_angle()*cos(theta)*track_wheel_circumference;
		//y_loc = original_y_loc + tracking_p.get_angle()*cos(theta)*track_wheel_circumference;
		
		/** 
		 * Delay so other processes can run
		 * 10 ms is the refresh rate of the Rotation sensors
		 */
		 pros::delay(200);
	} while (true) {
		pros::delay(10);
	}
}
