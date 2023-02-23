#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/error.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include <cctype>
#include <cmath>
#include <cstddef>
#include <string>
#include <sys/errno.h>
#include <type_traits>

// Define Ports for Motors
#define CHASSIS_L1_PORT 4
#define CHASSIS_L2_PORT 1
#define CHASSIS_L3_PORT 8
#define CHASSIS_L4_PORT 6
#define CHASSIS_R1_PORT 5
#define CHASSIS_R2_PORT 10
#define CHASSIS_R3_PORT 15
#define CHASSIS_R4_PORT 11
#define INTAKE_PORT 19
#define FLYWHEEL_PORT 2
#define ROLLER_PORT 18

// Define Ports for Sensors
#define ROLLER_SENSOR_PORT 1
#define TRACKING_SIDE_PORT 13
#define TRACKING_FORWARD_PORT 20
#define GYRO_PORT 9
#define COLOR_PORT 7

// Define Ports for sensors and pistons on the Analog Ports
#define INDEXER_PORT 2
#define ENDGAME_PORT 4
#define FLYWHEEL_POTENTIOMETER_PORT 3


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
static constexpr double start_theta = 270;
// The x coordinate of our alliance goal in millimeters
static constexpr int goal_x = 457;
// The y coordinate of our alliance goal in millimeters
static constexpr int goal_y = 457;


// Varaiables to keep track of the Location of the Robot
double x_loc = 1385, y_loc = 3238;
// Variable to keep track of the Orientation of the Robot
double theta;

// Variables to store the target speed for each side of the chassis
int left_target = 0, right_target = 0;

// variable to store the target speed
int flywheel_target = 0;



// Define Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// Define Motors
pros::Motor chassis_r1(CHASSIS_R1_PORT, true);
pros::Motor chassis_r2(CHASSIS_R2_PORT, true);
pros::Motor chassis_r3(CHASSIS_R3_PORT, true);
pros::Motor chassis_r4(CHASSIS_R4_PORT);
pros::Motor chassis_l1(CHASSIS_L1_PORT);
pros::Motor chassis_l2(CHASSIS_L2_PORT);
pros::Motor chassis_l3(CHASSIS_L3_PORT);
pros::Motor chassis_l4(CHASSIS_L4_PORT, true);
pros::Motor intake(INTAKE_PORT, true);
pros::Motor flywheel(FLYWHEEL_PORT, true);
pros::Motor roller(ROLLER_PORT);

// Define Pistons
pros::ADIDigitalOut indexer(INDEXER_PORT);
pros::ADIDigitalOut endgame(ENDGAME_PORT);
// Define Sensors
pros::Optical roller_sensor(ROLLER_SENSOR_PORT);
pros::Rotation tracking_side(TRACKING_SIDE_PORT);
pros::Rotation tracking_forward(TRACKING_FORWARD_PORT);
pros::ADIPotentiometer flywheel_potentiometer(FLYWHEEL_POTENTIOMETER_PORT);
pros::IMU gyro(GYRO_PORT);
pros::Optical color(COLOR_PORT);


/**
 * Gets the distance between two points
 * 
 * @param x0 the x coordinate of the first point
 * @param y0 the y coordinate of the first point
 * @param x1 the x coordinate of the second point
 * @param y1 the y coordinate of the second point
 */
double get_distance(double x0, double y0, double x1, double y1) {
	return sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1));
}


/**
 * Gets the distance from the robot to the goal
 *
 * @return the distance in mm between the robot and the goal
 */
double get_goal_distance() {
	return get_distance(x_loc, y_loc, goal_x, goal_y);
}

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

		pros::lcd::set_text(0, 
		"x: " + std::to_string((int)(x_loc*mm_to_inch)) + 
		"    y: " + std::to_string((int)(y_loc*mm_to_inch)) + 
		"    theta: " + std::to_string((int)theta));

		// Calculate flywheel speed based on position and angle
		flywheel_target = 0.04049 * get_goal_distance() + 97.61662;
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
	// Main control loop
	int left_speed = 0;
	int right_speed = 0;
	const int slew = 60;
	const int slew_f = 35;
	while (true) {
		if (chassis_l1.get_actual_velocity() + slew_f < left_target) {
			left_speed = chassis_l1.get_actual_velocity() + slew_f;
		} else if (chassis_l1.get_actual_velocity() - slew > left_target) {
			left_speed = chassis_l1.get_actual_velocity() - slew;
		} else {
			left_speed = left_target;
		}

		if (chassis_r1.get_actual_velocity() + slew_f < right_target) {
			right_speed = chassis_r1.get_actual_velocity() + slew_f;
		} else if (chassis_r1.get_actual_velocity() - slew > right_target) {
			right_speed = chassis_r1.get_actual_velocity() - slew;
		} else {
			right_speed = right_target;
		}

		chassis_l1.move_velocity(left_speed);
		chassis_l2.move_velocity(left_speed);
		chassis_l3.move_velocity(left_speed);
		chassis_l4.move_velocity(left_speed*3/5);
		chassis_r1.move_velocity(right_speed);
		chassis_r2.move_velocity(right_speed);
		chassis_r3.move_velocity(right_speed);
		chassis_r4.move_velocity(right_speed*3/5);
		// Delay for other tasks to run
		pros::delay(20);
	}
	return 0;
}

/**
 * Task to control flywheel speed using take back half algorithm
 */
int flywheel_task () {
	// take back half constant for getting close to desired speed
	int tbh = 6000;
	// the difference between the desired and actual speed
	int error;
	// the actual voltage to set the motor speed to
	int output = 0;
	// Control loop for flywheel
	while (true) {
		// calculate differences in desired speed
		error = flywheel_target - flywheel.get_actual_velocity();

		// accumulate voltage to get to good speed
		output += error;
		
		// If going to fast slow down some
		if (error < -flywheel_target/25) {
			output = (output+tbh)/2;
			tbh = output;
		}

		// Prevent exceeding maximum voltage to not cause errors in calculations
		if (output > 12000) output = 12000;

		// pros::lcd::set_text(5, "flywheel: " + std::to_string(flywheel.get_actual_velocity()));

		// Set Flywheel speed to calculated value
		// flywheel.move_voltage(output);

		flywheel.move_velocity(flywheel_target);

		// Delay so other processes can run
		pros::delay(10);
	}
	return 0;
}

void initialize() {
	
	pros::lcd::initialize();
	// Delay to allow calibration of sensors
	pros::delay(3000);
	// Initialize lcd for debugging
	


	pros::Task drive_task(drive);
	pros::Task odometry_task(odometry);
	pros::Task run_flywheel_task(flywheel_task);
}

/**
 * Turns to face specific orientation
 * Function is Blocking
 *
 * @param angle the angle that the robot should turn to
 */
void turn_to(double angle) {
	double error = -180 +fmod((angle+180-(fmod(theta, 360))),360);
	double prev_error;
	double total_error = 0;
	// The precision in degrees required to exit the loop
	const double threshold = 1;
	// The constants tuned for PID
	const double kp = 1, ki = 0.05, kd = 2;

	// PID control loop
	while (fabs(error) > threshold || fabs(prev_error) > threshold) {
		prev_error = error;
		error = -180 + fmod((angle + 180 - (fmod(theta, 360))), 360);
		total_error += (fabs(error) < 2 ? error : 0);

		pros::lcd::set_text(2, "error: " + std::to_string(error));

		left_target = -kp * error - kd * (error - prev_error) - ki * total_error;
		right_target = kp * error + kd * (error - prev_error) + ki * total_error;
		pros::delay(10);
	}
	// Zero out motors so the robot does not continue moving
	left_target = 0;
	right_target = 0;

}

/**
 * Function to turn to face goal
 * Calculates the angle to turn based on odometry and uses the built in turn function to face the goal
 */
void turn_to_goal() {
	pros::lcd::set_text(4, "x: " + std::to_string(x_loc - goal_x) + "    " + "x: " + std::to_string(y_loc - goal_y));
	pros::lcd::set_text(5, "target theta " + std::to_string((atan((y_loc - goal_y)/(goal_x - x_loc)) +(x_loc < goal_x ? pi : 0))*radian_to_degree));
	turn_to((atan((y_loc - goal_y)/(goal_x - x_loc)) +(x_loc < goal_x ? pi : 0))*radian_to_degree);
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

void competition_initialize() {}

/**
 * Drives the robot forward a given distance
 * This function is blocking
 * 
 * @param distance the distance to travel in mm
 */
void drive_forward(int distance) {

	// Get initial position 
	const int target = tracking_forward.get_position() + distance/track_wheel_size;
	// the difference between the desired position and the current distance
	double error = target - tracking_forward.get_position();
	double prev_error;
	// accumulate total error to correct for it
	double total_error = 0;
	// The precision in mm
	const double threshold = 1;
	// The constants tuned for PID
	const double kp = .001, ki = 0.005, kd = .000000001;
	// PID control loop
	while (fabs(error)*track_wheel_size > threshold || fabs(prev_error)*track_wheel_size > threshold) {
		prev_error = error;

		error = target - tracking_forward.get_position();
		
		total_error += (fabs(error) < 1000 ? error : 0);
		pros::lcd::set_text(1, "prev error: " + std::to_string((int)prev_error) + "       " + std::to_string(prev_error*track_wheel_size*mm_to_inch));
		pros::lcd::set_text(2, "error:      " + std::to_string((int)error) + "       " + std::to_string(error*track_wheel_size*mm_to_inch));
		pros::lcd::set_text(3, "speed: " + std::to_string(left_target));
		pros::lcd::set_text(4, "threshold: " + std::to_string(threshold/track_wheel_size));


		left_target = kp * error + kd * (error - prev_error) + ki * total_error;
		left_target = abs(left_target) > 180 ? 180*(left_target > 0 ? 1:-1) : left_target;
		right_target = left_target;
		pros::delay(10);
	}
	// Zero out motors so the robot does not continue moving
	left_target = 0;
	right_target = 0;
}

/**
 * Shoot the flywheel at the given speed a certain number of times
 * 
 * @param speed the speed the flywheel should shoot at
 * @param count the number of times the robot should shoot at that speed
 */
void shoot(int count) {
	// Wait until flywheel is at desired speed
	for (; count > 0; count--) {
		while (fabs(flywheel_target - flywheel.get_actual_velocity()) > 5) { pros::delay(10); }
		// Shoot
		indexer.set_value(true);
		pros::delay(100);
		indexer.set_value(false);
		pros::delay(300);
	}
	pros::delay(300);
}

/**
 *	Gets if the color of the roller optical sensor is red
 *	@return true if the color is red otherwise false
 */
bool is_red(double hue) {
	return color.get_hue() > 300 || color.get_hue() < 20;
}

/**
 * Autonomous roller function to change the roller from current color to new color
 */
void run_roller(){
	// Turn on flashlight
	color.set_led_pwm(100);
	
	// Drive forward into roller
	left_target = -20;
	right_target = -20;

	// counter to exit loop if taking too long
	int counter = 0;

	// get the start color of the roller
	bool start_color = is_red(color.get_hue());

	// while start color is not the current color
	while (start_color == is_red(color.get_hue()) && counter < 200) {
		// turn roller
		roller.move(70);
		// counter to break if stuck on screw
		counter++;
		// delay to allow other tasks to run
		pros::delay(10);
	}

	// Stops the intake, robot movement, and turns off flashlight
	roller.brake();
	
	// brake chassis
	left_target = 0;
	right_target = 0;

	// turn off flashlight
	color.set_led_pwm(0);
}

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
void autonomous() {	
//turns to goal and shoots
	turn_to(285);
	drive_forward(11*inch_to_mm);
	shoot(2);
//turns backwards to intake, angles next position
	turn_to(45);
//pick up the three discs and shoots
	intake.move_velocity(176);
//backwards movement
	drive_forward(-44*inch_to_mm);
	//aims at goal again
	turn_to(325);
	//gives downtime to move into position
	pros::delay(500);
	shoot(3);




	/**

	turn_to(315);
	drive_forward(44*inch_to_mm);
	shoot(3); 
	notes -
	knock over the nearby three stack and then grab them nad shoot at the goal
	if that doesnt work, grab the nearest three by the goal and shoot
	adjust the movement speed after function is fixed
	**/
	
}


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
	bool endgame_flag = false;
	// variable to speed up or slow down flywheel based on user needs
	int flywheel_offset = 0;
	int y_joystick = 0, x_joystick = 0;
	// Variables to calculate chassis speed when using arcade drive
	int power, turn;
	// Main Control Loop
	while (true) {
		if (controller.get_digital_new_press(DIGITAL_DOWN)) {
			pros::Task run_flywheel_task(flywheel_task);
		}


		// Debugging for the Intake
		if (controller.get_digital(DIGITAL_R1))
			intake = 127;
		else if (controller.get_digital(DIGITAL_R2))
			intake = -127;
		else
			intake = 0;

		/**
		 * When A is pressed starts a timer of 8 iterations until piston retracts
		 * Piston expands if timer value is positive
         * otherwise piston retracts
		 */
		if (controller.get_digital_new_press(DIGITAL_A)) 
			indexing_flag = 8;
		indexer.set_value(indexing_flag >= 0);
		indexing_flag -= 1;
		
		// Debugging prints for flywheel
		// pros::lcd::set_text(0, "angle: " + std::to_string((flywheel_potentiometer.get_angle())));
		// pros::lcd::set_text(1, "speed: " + std::to_string(flywheel.get_actual_velocity()));
		
		
		// Roller code

		if (controller.get_digital(DIGITAL_L1)) {
			roller = 127;
		} else if (controller.get_digital(DIGITAL_L2)) {
			roller = -127;
		} else {
			roller.brake();
		}

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
		y_joystick = controller.get_analog(ANALOG_LEFT_Y);
		x_joystick = controller.get_analog(ANALOG_LEFT_X);

		power = abs(y_joystick) > 8 ? (y_joystick-std::signbit(y_joystick)*8)*200/119 : 0;
		turn = abs(x_joystick) > 8 ? (x_joystick-std::signbit(x_joystick)*8)*200/119 : 0; 
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
		// left_target = abs(controller.get_analog(ANALOG_RIGHT_Y)) > 8 ? (controller.get_analog(ANALOG_RIGHT_Y)-8)*200/119 : 0; 
		// right_target = abs(controller.get_analog(ANALOG_LEFT_Y)) > 8 ? (controller.get_analog(ANALOG_LEFT_Y)-8)*200/119 : 0; 
		
		


		// Turn to goal when X pressed on the controller
		if (controller.get_digital_new_press(DIGITAL_X)) {
			turn_to_goal();
		}

		if (controller.get_digital(DIGITAL_UP) && controller.get_digital(DIGITAL_RIGHT) ) {
			endgame_flag = !endgame_flag;

			endgame.set_value(endgame_flag);
			pros::delay(500);
		}
		

		
		

		// Delay so other processes can run
		pros::delay(10);
	}
}