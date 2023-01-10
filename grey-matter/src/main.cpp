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
#define CHASSIS_L1_PORT 3
#define CHASSIS_L2_PORT 1
#define CHASSIS_L3_PORT 8
#define CHASSIS_R1_PORT 5
#define CHASSIS_R2_PORT 10
#define CHASSIS_R3_PORT 15
#define INTAKE_PORT 19
#define FLYWHEEL_A_PORT 19
#define FLYWHEEL_PORT 19
#define ROLLER_PORT 19

// Define Ports for Sensors
#define ROLLER_SENSOR_PORT 1
#define TRACKING_SIDE_PORT 13
#define TRACKING_FORWARD_PORT 20
#define GYRO_PORT 9

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
// The angle the robot is facing
static constexpr double start_theta = 0;
// The x coordinate of our alliance goal in millimeters
static constexpr int goal_x = 457;
// The y coordinate of our alliance goal in millimeters
static constexpr int goal_y = 457;


// Varaiables to keep track of the Location of the Robot
double x_loc = 3467, y_loc = 1524;
// Variable to keep track of the Orientation of the Robot
double theta = 0;


// Variables to store the target speed for each side of the chassis
int left_target = 0, right_target = 0;


// Define Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// Define Motors
pros::Motor chassis_r1(CHASSIS_R1_PORT, true);
pros::Motor chassis_r2(CHASSIS_R2_PORT, true);
pros::Motor chassis_r3(CHASSIS_R3_PORT, true);
pros::Motor chassis_l1(CHASSIS_L1_PORT);
pros::Motor chassis_l2(CHASSIS_L2_PORT);
pros::Motor chassis_l3(CHASSIS_L3_PORT);
pros::Motor intake(INTAKE_PORT, true);
pros::Motor flywheel(FLYWHEEL_PORT);
pros::Motor roller(ROLLER_PORT);
pros::Motor flywheel_angle(FLYWHEEL_A_PORT);
// Define Pistons
pros::ADIDigitalOut indexer(INDEXER_PORT);
// Define Sensors
pros::Optical roller_sensor(ROLLER_SENSOR_PORT);
pros::Rotation tracking_side(TRACKING_SIDE_PORT);
pros::Rotation tracking_forward(TRACKING_FORWARD_PORT);
pros::ADIPotentiometer flywheel_angles(FLYWHEEL_ANGLE_PORT);
pros::IMU gyro(GYRO_PORT);


/**
 * Task keeps track of the location and heading of the robot
 */
int odometry() {
	// Sets gyro rotation to starting angle of robot
	gyro.set_rotation(start_theta);
	// Gets the position of the tracking wheels
	int track_side = tracking_side.get_position(), track_forward = tracking_forward.get_position();
	// Stores the previous iteration of tracking wheel positions
	int prev_track_side = track_side, prev_track_forward = track_forward;
	// Sensor values for the movement of the tracking wheels.
	double mm_forward = 0, mm_side = 0;
	// Odometry control loop
	while (true) {
		// Get the sensor values for the tracking wheels and inertial sensor
		track_forward = tracking_forward.get_position();
		track_side = tracking_side.get_position();
		// track_theta = gyro.get_rotation();

		// Converts sensor values to respective standardized units (radians and mm)
		mm_forward = (prev_track_forward - track_forward)*track_wheel_size;
		mm_side = (track_side - prev_track_side)*track_wheel_size;

		// Converts the local movements to the global position
		y_loc -= cos(theta*degree_to_radian)*mm_side + sin(theta*degree_to_radian)*mm_forward;
		x_loc += cos(theta*degree_to_radian)*mm_forward - sin(theta*degree_to_radian)*mm_side;
		theta = gyro.get_rotation();

		// Stores current sensor values to previous variables for use in next iteration
		prev_track_forward = track_forward;
		prev_track_side = track_side;

		// Delay so that other tasks can run
		pros::delay(10);
	}
	return 0;
}


/**
 * Task to drive the robot
 * 
 * Uses slew to prevent voltage from changing to rapidly
 * Uses PID to get to desired speed as quickly as possible
 */
int drive () {
	// The actual voltage for the left and right sides of the chassis
	int voltage_al = 0, voltage_ar = 0;
	// The target speed for the left and right sides of the chassis
	int speed_l, speed_r;
	// The voltage needed to maintain the speed of the left or right side of the chassis
	int voltage_ml, voltage_mr;
	// The target voltage for the left and right side of the chassis
	int voltage_tl, voltage_tr;
	// The error, previous error and total error for the left side of the chassis
	int error_l, prev_error_l = 0, total_error_l = 0;
	// The error, previous error and total error for the right side of the chassis
	int error_r, prev_error_r = 0, total_error_r = 0;
	// The constant for the pid loop
	int kp = 20, kd = 15, ki = 1;
	// The maximum slew rate
	int slew = 500;
	// Main control loop
	while (true) {
		// Set speed of left and right side of chassis based on tank drive controls 
		speed_l = left_target*210/127;
		speed_r = right_target*210/127;

		// Calculate the error between the target velocity and true velocity
		error_l = speed_l - chassis_l1.get_actual_velocity();
		error_r = speed_r - chassis_r1.get_actual_velocity();

		// Accumulate small errors to correct for differences between theoretical and
		// experimental voltage to velocity correlations
		if (abs(error_l) < 5) total_error_l += error_l;
		if (abs(error_r) < 5) total_error_r += error_r;

		// Calculate the voltage needed to maintain the desired velocity
		voltage_ml = speed_l*50 + (speed_l < 0 ? -1250 : 1250);
		voltage_mr = speed_r*50 + (speed_r < 0 ? -1250 : 1250);

		// If voltage would be too small to move motor then set voltage to zero
		voltage_ml = abs(speed_l) > 15 ? voltage_ml : 0;
		voltage_mr = abs(speed_r) > 15 ? voltage_mr : 0;

		// Calculate Target velocity using the maintenance velocity as a base line and a PID
		// controller to reach that velocity
		voltage_tl = voltage_ml + kp*error_l + kd*(prev_error_l - error_l);
		voltage_tr = voltage_mr + kp*error_r + kd*(prev_error_r - error_r);

		// Set previous error to current error for next iteration
		prev_error_l = error_l;
		prev_error_r = error_r;

		// Move actual voltage towards target voltage by an increment no greater than the slew rate
		if (abs(voltage_al - voltage_tl) <= slew) voltage_al = voltage_tl;
		else voltage_al += abs(voltage_al - voltage_tl)/(voltage_tl - voltage_al)*slew;
		if (abs(voltage_ar - voltage_tr) <= slew) voltage_ar = voltage_tr;
		else voltage_ar += abs(voltage_ar - voltage_tr)/(voltage_tr - voltage_ar)*slew;

		// Make sure actual voltage lies between -12000 and 12000 mv
		if (voltage_al > 12000) voltage_al = 12000;
		else if (voltage_al < -12000) voltage_al = -12000;
		if (voltage_ar > 12000) voltage_ar = 12000;
		else if (voltage_ar < -12000) voltage_ar = -12000;

		// Set motor to actual voltage
		chassis_l1.move_voltage(voltage_al);
		chassis_l2.move_voltage(voltage_al);
		chassis_l3.move_voltage(voltage_al);
		chassis_r1.move_voltage(voltage_ar);
		chassis_r2.move_voltage(voltage_ar);
		chassis_r3.move_voltage(voltage_ar);

		// Delay for other tasks to run
		pros::delay(10);
	}
	return 0;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Delay to allow calibration of sensors
	pros::delay(3000);
	// Initialize lcd for debugging
	pros::lcd::initialize();

	flywheel_angle.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}


/**
 * Function for testing purposes to turn to face goal
 * 
 * Used to turn towards goal
 * Used to test odometry
 */
void turn_to_goal() {
	// Difference in y values between the robot and goal in mm
	double y = y_loc - goal_y;
	// Difference in x values between the robot and goal in mm
	double x = x_loc - goal_x;

	// Heading between 0 and 2 pi for the direction that the robot needs to be facing at the given
	// point to be facing the goal
	double target_heading = fmod(((x > 0 ? 2*pi : 3*pi) - atan(y/x)),2*pi);

	// Debugging values
	pros::lcd::set_text(0, std::to_string(target_heading));
	pros::lcd::set_text(1, std::to_string((std::fmod(std::fmod(theta, 360)+360, 360)*degree_to_radian)));
	pros::lcd::set_text(2, std::to_string(fabs((std::fmod(std::fmod(theta, 360)+360, 360)*degree_to_radian - target_heading))));
	
	// Control loop to reach desired target
	// Converts theta to a positive value between 0 and 2pi
	// while difference is less than 0.2 radians (17 degrees)
	while(fabs((std::fmod(std::fmod(theta,360)+360,360)*degree_to_radian-target_heading))>0.3){
		// Debugging
		pros::lcd::set_text(1, std::to_string((std::fmod(std::fmod(theta, 360)+360, 360)*degree_to_radian)));
		pros::lcd::set_text(2, std::to_string(fabs((std::fmod(std::fmod(theta, 360)+360, 360)*degree_to_radian - target_heading))));
		
		// Turn left or right based on the difference in angles
		if (theta*degree_to_radian < target_heading) {
			left_target = 50;
			right_target = -50;
		} else {
			left_target = -50;
			right_target = 50;
		}

		// Delay to let other tasks run
		pros::delay(10);
	}
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
	// Flag to set the position of the indexing piston
	int indexing_flag = 0;
	// Main Control Loop

	pros::Task drive_task(drive);
	pros::Task odometry_task(odometry);

	while (true) {
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
		
		if (controller.get_digital(DIGITAL_UP)) {
			flywheel_angle = 50;
			pros::lcd::set_text(0, "15");
		} else if (controller.get_digital(DIGITAL_DOWN)) {
			pros::lcd::set_text(1, "-15");
			flywheel_angle = -50;
		} else {
			flywheel_angle.brake();
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
		int power = controller.get_analog(ANALOG_LEFT_Y); 
		int turn = controller.get_analog(ANALOG_LEFT_X); 
		right_target = power + turn; 
		left_target = power - turn; 
 
		/** 
         * Base Tank Controls 
         *
         * Sets left side of Chassis to left joystick value
         * Sets right side of chassis to right joystick value
         * 
         * Joystick has a small deadzone to prevent accidental movements when 
         * the joysticks are not perfectly centered
         */ 
		// left_target = abs(controller.get_analog(ANALOG_LEFT_Y)) > 8 ? controller.get_analog(ANALOG_LEFT_Y) : 0; 
		// right_target = abs(controller.get_analog(ANALOG_RIGHT_Y)) > 8 ? controller.get_analog(ANALOG_RIGHT_Y) : 0; 
		
	
		// Turn to goal when A pressed on the controller
		if (controller.get_digital(DIGITAL_A)) {
			turn_to_goal();
		}

		// Delay so other processes can run
		pros::delay(10);
	}
}
