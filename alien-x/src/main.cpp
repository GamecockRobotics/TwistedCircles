#include "main.h"
#include "gui.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.h"
#include <string>


//defining motors ports
#define CHASSIS_L1_PORT 11
#define CHASSIS_L2_PORT 12
#define CHASSIS_L3_PORT 13
#define CHASSIS_L4_PORT 14
#define CHASSIS_R1_PORT 15
#define CHASSIS_R2_PORT 16
#define CHASSIS_R3_PORT 17
#define CHASSIS_R4_PORT 18
#define CATA_L_PORT 9
#define CATA_R_PORT 10
#define INTAKE1_PORT 4
#define INTAKE2_PORT 5
#define COLOR_PORT 8
#define ROLLER1_PORT 2
#define ROLLER2_PORT 3



//defining sensor ports
//#define GYRO_PORT 11
//#define OPTICAL_PORT 12

//declaring motors
pros::Motor chassis_L1(CHASSIS_L1_PORT,true);
pros::Motor chassis_L2(CHASSIS_L2_PORT,true);
pros::Motor chassis_L3(CHASSIS_L3_PORT,true);
pros::Motor chassis_L4(CHASSIS_L4_PORT);
pros::Motor chassis_R1(CHASSIS_R1_PORT);
pros::Motor chassis_R2(CHASSIS_R2_PORT);
pros::Motor chassis_R3(CHASSIS_R3_PORT);
pros::Motor chassis_R4(CHASSIS_R4_PORT, true);
pros::Motor intake_1(INTAKE1_PORT);
pros::Motor intake_2(INTAKE2_PORT,true);
// pros::Motor roller(ROLLER_PORT);
pros::Motor catapult_L(CATA_L_PORT);
pros::Motor catapult_R(CATA_R_PORT, true);
pros::Motor roller_1(ROLLER1_PORT);
pros::Motor roller_2(ROLLER2_PORT,true);


//Declaring sensors and pneumatics
//pros::Imu gyro (GYRO_PORT);
//pros::ADIDigitalOut launcher(LAUNCHER_PORT);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Optical color(COLOR_PORT);


enum intakeDirection {stopped, intake, outtake};
intakeDirection intakeState = stopped;

enum TurnType {left, right};
enum DirectionType {forward, backward};

//Declaring variables
int catapultTarget = 0;

//TODO Make Function that can autoaim power

//TODO Odometry 

//TODO figure out how tracking wheels are positioned

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */




pros::Motor catapult(CATA_L_PORT);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	// intake_1.set_brake_mode(MOTOR_BRAKE_COAST);
	// intake_2.set_brake_mode(MOTOR_BRAKE_COAST);

	
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


int driveTask(){
	while(true){
		//Base Tank Controls
		int left = controller.get_analog(ANALOG_LEFT_Y);
		int right = controller.get_analog(ANALOG_RIGHT_Y);
		if(abs(right) > 5){
			right = right -5 * (127/122);
			chassis_R1.move(right);
			chassis_R2.move(right);
			chassis_R3.move(right);
			chassis_R4.move(right);
		} else {
			chassis_R1.brake();
			chassis_R2.brake();
			chassis_R3.brake();
			chassis_R4.brake();
		}
		if(abs(left) > 5){
			left = left - 5 * (127/122);
			chassis_L1.move(left);
			chassis_L2.move(left);
			chassis_L3.move(left);
			chassis_L4.move(left);
		} else {
			chassis_L1.brake();
			chassis_L2.brake();
			chassis_L3.brake();
			chassis_L4.brake();
		}

	
	}
	
	return 0;
}

void shoot(){
	if (catapultTarget == 0) {
		catapultTarget = 127;
	} else if (catapultTarget == 127 && catapult_L.get_efficiency() < 5) {
		catapultTarget = -127;
	} else if (catapultTarget == -127 && catapult_L.get_efficiency() < 5) {
		catapultTarget = 0;
	}
	catapult_L = catapultTarget;
	catapult_R = catapultTarget;
}

/**
 *	Gets if the color of the roller optical sensor is red
 *	@return true if the color is red otherwise false
 */
bool is_red(double hue) {
	return color.get_hue() < 300 || color.get_hue() < 25;
}

void chassis_L(int speed){
	chassis_L1 = speed;
	chassis_L2 = speed;
	chassis_L3 = speed;
	chassis_L4 = speed;
	return;
}
void chassis_R(int speed){
	chassis_R1 = speed;
	chassis_R2 = speed;
	chassis_R3 = speed;
	chassis_R4 = speed;
	return;
}

/**
 * Autonomous roller function to change the roller from current color to new color
 */
void run_roller(){
	// Turn on flashlight
	color.set_led_pwm(100);

	chassis_R(30);	
	chassis_L(30);	


	// counter to exit loop if taking too long
	int counter = 0;

	// get the start color of the roller
	bool start_color = is_red(color.get_hue());
	pros::lcd::set_text(2, std::to_string(start_color));
	counter = 0;

	// while start color is not the current color
	while (start_color == is_red(color.get_hue()) && counter < 200) {
		pros::lcd::set_text(3, std::to_string(start_color));
		// turn roller
		roller_1.move(40);
		roller_2.move(40);
		// counter to break if stuck on screw
		counter++;
		// delay to allow other tasks to run
		pros::delay(10);
		
	}

	// Stops the intake, robot movement, and turns off flashlight
	roller_1.brake();
	roller_2.brake();

	chassis_L(0);	
	chassis_R(0);	


	// turn off flashlight
	color.set_led_pwm(0);
	pros::delay(10);

	return;
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
 void autonomous() {
	pros::c::delay(500);
	run_roller();


	
}

void opcontrol() {
	bool intakeFlag = true;


	
	pros::Task drive(driveTask);
	while (true) {
		// if(controller.get_digital(DIGITAL_A) || catapultTarget != 0){
		// 	shoot();
		// 	intakeFlag = false;
		// }

		if(controller.get_digital(DIGITAL_R1)){
			catapult_L= 127;
			catapult_R= 127;
		} else if (controller.get_digital(DIGITAL_R2)) {
			catapult_L= -127;
			catapult_R= -127;
		} else {
			catapult_L = 0;
			catapult_R = 0;
		}

		if(controller.get_digital(DIGITAL_L1)){
			intake_1 = 127;
			intake_2 = 127;
		} else if(controller.get_digital(DIGITAL_L2)){
			intake_1 = -127;
			intake_2 = -127;
		} else {
			intake_1 = 0;
			intake_2 = 0;
		}

	}
}
