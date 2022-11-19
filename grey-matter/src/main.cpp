#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

// Define Ports for Motors
#define CHASSIS_L1_PORT 1
#define CHASSIS_L2_PORT 2
#define CHASSIS_L3_PORT 3
#define CHASSIS_R1_PORT 4
#define CHASSIS_R2_PORT 5
#define CHASSIS_R3_PORT 6
#define INTAKE_PORT 7
#define FLYWHEEL_A1_PORT 8
#define FLYWHEEL_A2_PORT 9
#define FLYWHEEL_PORT 10
#define ROLLER_PORT 11

// Define Ports for Sensors
#define DISTANCE_B1_PORT 13
#define DISTANCE_B2_PORT 14
#define DISTANCE_L_PORT 15
#define DISTANCE_R_PORT 16
#define ROLLER_SENSOR_PORT 17
#define TRACKING_B_PORT 18
#define TRACKING_F_PORT 19
#define TRACKING_P_PORT 20

// Define Ports for sensors and pistons on the Analog Ports
#define FLYWHEEL_ANGLE_PORT 1
#define DISTANCE_LP_PORT 2
#define DISTANCE_RP_PORT 3
#define INDEXER_PORT 4
#define ENDGAME_PORT 5


/**
 * Define physical Dimensions of Robot
 * All measurements are in millimeters and degrees
 */

// Distance from Top Left Sensor to Measuring Point of Robot
int const tl_sensor_x = 0;
int const tl_sensor_y = 0;
// Distance from Top Right Sensor to Measuring Point of Robot
int const tr_sensor_x = 0;
int const tr_sensor_y = 0;
// Distance from Bottom Left Sensor to Measuring Point of Robot
int const bl_sensor_x = 0;
int const bl_sensor_y = 0;
// Distance from Bottom Right Sensor to Measuring Point of Robot
int const br_sensor_x = 0;
int const br_sensor_y = 0;
// Distance between Parallel Tracking Wheels 
int const track_sensor_separation = 0;
// Measurement of tracking Wheel Circumference
int const track_wheel_circumference = 220;

// Varaiables to keep track of the location and orientation of the robot
int x_loc, y_loc;
float theta;

// Define Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// Define Motors
pros::Motor chassis_r1(CHASSIS_R1_PORT);
pros::Motor chassis_r2(CHASSIS_R2_PORT);
pros::Motor chassis_r3(CHASSIS_R3_PORT);
pros::Motor chassis_l1(CHASSIS_L1_PORT, true);
pros::Motor chassis_l2(CHASSIS_L2_PORT, true);
pros::Motor chassis_l3(CHASSIS_L3_PORT, true);
pros::Motor intake(INTAKE_PORT);
pros::Motor flywheel(FLYWHEEL_PORT);
pros::Motor roller(ROLLER_PORT);
pros::Motor flywheel_a1(FLYWHEEL_A1_PORT);
pros::Motor flywheel_a2(FLYWHEEL_A2_PORT);
// Define Pistons
// Define Sensors
pros::Distance distance_b1(DISTANCE_B1_PORT);
pros::Distance distance_b2(DISTANCE_B2_PORT);
pros::Distance distance_l(DISTANCE_L_PORT);
pros::Distance distance_r(DISTANCE_R_PORT);
pros::Optical roller_sensor(ROLLER_SENSOR_PORT);
pros::Rotation tracking_b(TRACKING_B_PORT);
pros::Rotation tracking_f(TRACKING_F_PORT);
pros::Rotation tracking_p(TRACKING_P_PORT);
pros::ADIPotentiometer flywheel_angle(FLYWHEEL_ANGLE_PORT);
pros::ADIPotentiometer distance_lp(DISTANCE_LP_PORT);
pros::ADIPotentiometer distance_rp(DISTANCE_RP_PORT);


/**
 * Calibrates the Location of the robot
 */
void calibrate_location() {
	
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	calibrate_location();
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
	// Variables to store the target speed for each side of the chassis
	int leftSpeed = 0, rightSpeed = 0;
	// Main Control Loop
	while (true) {
		
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
		leftSpeed = abs(controller.get_analog(ANALOG_LEFT_Y)) > 5 ? controller.get_analog(ANALOG_LEFT_Y) : 0; 
		rightSpeed = abs(controller.get_analog(ANALOG_RIGHT_Y)) > 5 ? controller.get_analog(ANALOG_RIGHT_Y) : 0; 

        // Set Chassis Motors to Target Speed
		chassis_l1 = leftSpeed;
		chassis_l2 = leftSpeed;
		chassis_l3 = leftSpeed;
		chassis_r1 = rightSpeed;
		chassis_r2 = rightSpeed;
		chassis_r3 = rightSpeed;

		/** 
		 * Delay so other processes can run
		 * 10 ms is the refresh rate of the Rotation sensors
		 */
		pros::delay(10);
	}
}
