#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/error.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cctype>
#include <cmath>
#include <cstddef>
#include <string>
#include <sys/errno.h>
#include <type_traits>

// Define Ports for Motors
#define CHASSIS_L1_PORT 3
#define CHASSIS_L2_PORT 1
#define CHASSIS_L3_PORT 8
#define CHASSIS_R1_PORT 5
#define CHASSIS_R2_PORT 10
#define CHASSIS_R3_PORT 15
#define INTAKE_PORT 19
#define FLYWHEEL_A_PORT 14
#define FLYWHEEL_PORT 2
#define ROLLER_PORT 18

// Define Ports for Sensors
#define ROLLER_SENSOR_PORT 1
#define TRACKING_SIDE_PORT 13
#define TRACKING_FORWARD_PORT 20
#define GYRO_PORT 9

// Define Ports for sensors and pistons on the Analog Ports
#define INDEXER_PORT 2
#define ENDGAME_PORT 1
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
static constexpr double start_theta = -90;
// The x coordinate of our alliance goal in millimeters
static constexpr int goal_x = 457;
// The y coordinate of our alliance goal in millimeters
static constexpr int goal_y = 457;


// Varaiables to keep track of the Location of the Robot
double x_loc = 1378, y_loc = 3480;
// Variable to keep track of the Orientation of the Robot
double theta;

// Variables to store the target speed for each side of the chassis
int left_target = 0, right_target = 0;

// variable to store the target speed
int flywheel_target;



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
pros::Motor flywheel(FLYWHEEL_PORT, true);
pros::Motor flywheel_angle(FLYWHEEL_A_PORT, true);
pros::Motor roller(ROLLER_PORT);
// Define Pistons
pros::ADIDigitalOut indexer(INDEXER_PORT);
// Define Sensors
pros::Optical roller_sensor(ROLLER_SENSOR_PORT);
pros::Rotation tracking_side(TRACKING_SIDE_PORT);
pros::Rotation tracking_forward(TRACKING_FORWARD_PORT);
pros::ADIPotentiometer flywheel_potentiometer(FLYWHEEL_POTENTIOMETER_PORT);
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

		pros::lcd::set_text(0, "x: " + std::to_string(x_loc) + " y: " + std::to_string(y_loc));
		pros::lcd::set_text(1, "theta: " + std::to_string(theta));
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
	const int kp = 25, kd = 20, ki = 1;
	// The maximum slew rate; variable based on direction robot is traveling to prevent tipping
	int slew;
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
		slew = std::signbit(voltage_al - voltage_tl) && std::signbit(voltage_ar - voltage_tr) ? 600 :1600;

		if (abs(voltage_al - voltage_tl) <= slew) voltage_al = voltage_tl;
		else voltage_al += std::signbit(voltage_al - voltage_tl) ? slew : -slew;
		if (abs(voltage_ar - voltage_tr) <= slew) voltage_ar = voltage_tr;
		else voltage_ar += std::signbit(voltage_ar - voltage_tr) ? slew : -slew;

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
 * Gets the distance between two points
 * 
 * @param x0 the x coordinate of the first point
 * @param y0 the y coordinate of the first point
 * @param x1 the x coordinate of the second point
 * @param y1 the y coordinate of the second point
 */
double get_distance(double x0, double y0, double x1, double y1) {
	return sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1)) * ((x0 < x1 || (x0 == x1 &&y0<y1)) ? -1 : 1);
}

double get_distance_drive_forward(double x0, double y0, double x1, double y1) {
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
		// // Move the angle up while intaking 
		// if (intake.get_actual_velocity() > 50) {
		// 	if (flywheel_potentiometer.get_angle() < 103) 
		// 		flywheel_angle.brake();
		// 	else
		// 		flywheel_angle = 50;
		// // Move the angle down on the down button press
		// } else if (controller.get_digital(DIGITAL_DOWN))
		// 	flywheel_angle = -50;
		// else 
		// 	flywheel_angle.brake();

		// Move angle on manual control
		if (controller.get_digital(DIGITAL_DOWN)) {
			flywheel_angle = -50;
		}
		else if (controller.get_digital(DIGITAL_UP)) {
			flywheel_angle = 50;
		}
		else {
			flywheel_angle.brake();
		}

		// calculate differencec in desired speed
		error = flywheel_target - flywheel.get_actual_velocity();

		// accumulate voltage to get to good speed
		output += error;
		
		// If going to fast slow down some
		if (error < -5) {
			output = (output+tbh)/2;
			tbh = output;
		}

		// Prevent exceeding maximum voltage to not cause errors in calculations
		if (output > 12000) output = 12000;

		pros::lcd::set_text(5, "flywheel: " + std::to_string(flywheel.get_actual_velocity()));

		// Set Flywheel speed to calculated value
		flywheel.move_voltage(output);

		// Delay so other processes can run
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


	pros::Task drive_task(drive);
	pros::Task odometry_task(odometry);
	pros::Task run_flywheel_task(flywheel_task);


	flywheel_angle.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	
}

/**
 * Turn desired angle
 * 
 * Function is blocking
 * @param deg the number of degrees to turn
 */
void turn( double deg) {
	// Get initial position 
	double initial_angle = gyro.get_rotation();

	// the difference between the desired position and the current distance
	double error = deg, prev_error;
	// accumulate total error to correct for it, 
	// If the error is really small, start with a larger total error because kp will not do enough
	double total_error = fabs(deg) < 10 ? deg*50 : 0;
	// The precision in degrees required to exit the loop
	const double threshold = 1;
	// The constants tuned for PID
	const double kp = 2.25, ki = 0.05, kd = 2;

	
	// PID control loop
	while (fabs(error) > threshold || fabs(prev_error) > threshold) {
		prev_error = error;
		error = deg - (gyro.get_rotation() - initial_angle);
		total_error += (fabs(error) < 2 ? error : 0);

		left_target = -kp * error - kd * (error - prev_error) - ki * total_error;
		right_target = kp * error + kd * (error - prev_error) + ki * total_error;
		pros::delay(10);
		if (controller.get_digital(DIGITAL_B))
			break;
	}
	// Zero out motors so the robot does not continue moving
	left_target = 0;
	right_target = 0;
}

/**
 * Turns to face specific orientation
 * Function is Blocking
 *
 * @param angle the angle that the robot should turn to
 */
void turn_to_angle(double angle) {
	turn(fmod(fmod(fmod(fmod(theta, 360)+360,360) - angle,180)+180,180));
}

/**
 * Function to turn to face goal
 * Calculates the angle to turn based on odometry and uses the built in turn function to face the goal
 */
void turn_to_goal() {
	// Difference in y values between the robot and goal in mm
	double y = y_loc - goal_y;
	// Difference in x values between the robot and goal in mm
	double x = x_loc - goal_x;

	pros::lcd::set_text(3, "angle to face: " + std::to_string(atan(y/x)));
	pros::lcd::set_text(4, "angle to face: " + std::to_string(atan(y/x)*radian_to_degree));

	// Heading between -180 and 180 for the direction that the robot needs to be facing at the given
	// point to be facing the goal
	double target_theta = fmod(((x > 0 ? 0 : pi) - atan((y_loc - goal_y)/(x_loc - goal_x))),2*pi)*radian_to_degree - fmod(theta, 360);

	turn(fmod(target_theta, 360));

	// Theoretically correct though untested
	// turn_to_angle(((x > 0 ? 0 : pi) - atan((y_loc - goal_y)/(x_loc - goal_x))));

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
 * Drives the robot forward a given distance
 * This function is blocking
 * 
 * @param distance the distance to travel in mm
 */
void drive_forward(int distance) {
	int count = 0;

	// Get initial position 
	const int target_x = x_loc + distance*cos(theta*degree_to_radian);
	const int target_y = y_loc + distance*sin(theta*degree_to_radian);
	const double target_theta = gyro.get_rotation();

	pros::lcd::set_text(3, "X " + std::to_string(x_loc) + " target " + std::to_string(target_x));
	pros::lcd::set_text(4, "Y " + std::to_string(y_loc) + " target " + std::to_string(target_y));

	// the difference between the desired position and the current distance
	double dist_error = distance;
	double prev_dist_error;
	// accumulate total error to correct for it
	double total_dist_error = 0, total_theta_error = 0;
	// The precision in degrees required to exit the loop
	const double threshold = 15;
	// The constants tuned for PID
	const double kp = .4, ki = 0.001, kd = .5;
	const double kai = .05;
	// PID control loop
	while ((fabs(dist_error) > threshold || fabs(prev_dist_error) > threshold || abs(left_target) > 15 || abs(right_target) > 15) && count < 600) {
		count ++;
		pros::lcd::set_text(6, "I'm in pid");
		prev_dist_error = dist_error;

		dist_error = get_distance_drive_forward(x_loc, y_loc, target_x, target_y);

		if (fabs(target_x - x_loc) > 5 && x_loc < target_x) {
			dist_error *= -1;
		}		
		if (fabs(target_y - y_loc) > 5 && y_loc < target_y) {
			dist_error *= -1;
		}


		pros::lcd::set_text(7, "error: " + std::to_string(dist_error));
		
		total_dist_error += (fabs(dist_error) < 50 ? dist_error : 0);
		total_theta_error += target_theta - gyro.get_rotation();
		
		left_target = kp * dist_error + kd * (dist_error - prev_dist_error) + ki * total_dist_error;
		right_target = kp * dist_error + kd * (dist_error - prev_dist_error) + ki * total_dist_error;
		pros::delay(10);
	}
	pros::lcd::set_text(6, "I'm out of pid");
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
void shoot(int speed, int count) {
	// Set flywheel target to desired speed
	flywheel_target = speed;
	// Wait until flywheel is at desired speed
	while (fabs(flywheel_target - flywheel.get_actual_velocity()) > 5) { pros::delay(10); }
	// Shoot
	indexer.set_value(true);
	pros::delay(100);
	indexer.set_value(false);
	pros::delay(300);

	// For any following shots
	for (; count > 1; count--) {
		// Set flywheel to maximum speed
		flywheel_target = 220;
		// Wait until flywheel reaches desired speed
		while (flywheel.get_actual_velocity() < speed) { pros::delay(10); }
		// shoot
		indexer.set_value(true);
		pros::delay(100);
		indexer.set_value(false);
	pros::delay(300);
	}
	// make sure flywheel ends at speed passed into the function
	flywheel_target = speed;
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competitiofn testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	drive_forward(24*inch_to_mm);
	turn_to_goal();
	shoot(190, 2);

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
	// variable to speed up or slow down flywheel based on user needs
	int flywheel_offset = 0;
	// Variables to calculate chassis speed when using arcade drive
	int power, turn;
	// Main Control Loop
	while (true) {
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
		


        // Allow user to manually adjust flywheel speed
		// if (controller.get_digital_new_press(DIGITAL_LEFT)) { 
		// 	flywheel_offset -= 1;
		// }
        // if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
		// 	flywheel_offset += 1;
		// }

		// Calculate flywheel speed based on position and angle
		flywheel_target = 0.024866 * get_goal_distance() + 3.172274 * flywheel_potentiometer.get_angle() + -193.229168 +flywheel_offset;
		
		
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
		// power = controller.get_analog(ANALOG_LEFT_Y); 
		// turn = controller.get_analog(ANALOG_LEFT_X); 
		// right_target = power + turn; 
		// left_target = power - turn; 
 
		/** 
         * Base Tank Controls 
         *
         * Sets left side of Chassis to left joystick value
         * Sets right side of chassis to right joystick value
         * 
         * Joystick has a small deadzone to prevent accidental movements when 
         * the joysticks are not perfectly centered
         */ 
		left_target = abs(controller.get_analog(ANALOG_LEFT_Y)) > 8 ? -controller.get_analog(ANALOG_LEFT_Y) : 0; 
		right_target = abs(controller.get_analog(ANALOG_RIGHT_Y)) > 8 ? -controller.get_analog(ANALOG_RIGHT_Y) : 0; 
		
		// Non tilt correcting code
		// float leftPower = controller.get_analog(ANALOG_LEFT_Y);
		// float rightPower = controller.get_analog(ANALOG_RIGHT_Y);
		// if (fabs(leftPower) > 8) {
		// 	chassis_l1 = -leftPower;
		// 	chassis_l2 = -leftPower;
		// 	chassis_l3 = -leftPower;
		// } else {
		// 	chassis_l1 = 0;
		// 	chassis_l2 = 0;
		// 	chassis_l3 = 0;
		// }
		// if (fabs(rightPower) > 8) {
		// 	chassis_r1 = -rightPower;
		// 	chassis_r2 = -rightPower;
		// 	chassis_r3 = -rightPower;
		// } else {
		// 	chassis_r1 = 0;
		// 	chassis_r2 = 0;
		// 	chassis_r3 = 0;
		// }


		// Turn to goal when X pressed on the controller
		if (controller.get_digital(DIGITAL_X)) {
			turn_to_goal();
		}


		

		// Delay so other processes can run
		pros::delay(10);
	}
}