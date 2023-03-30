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

int OdmentryTest(){
	gyro.set_rotation(start_theta);

	//get distance travelled
	int prev_forward = tracking_forward.get_position(), prev_side = tracking_side.get_position();
	
	double distanceX = 0, distanceY = 0;

	while(true){
		
		theta = gyro.get_rotation();

		x_loc += cos(theta) * tracking_forward.get_position() + sin(theta) * tracking_side.get_position();
		y_loc += sin(theta) * tracking_forward.get_position() + cos(theta) * tracking_side.get_position();

		distanceY = (prev_forward - y_loc);
		distanceX = (prev_side - x_loc);

		prev_forward = y_loc;
		prev_side = x_loc;


		
	}
	
	

}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(2);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;

		pros::delay(20);
	}
}
