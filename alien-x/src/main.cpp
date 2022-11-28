#include "main.h"
#include "gui.h"
#include "pros/motors.hpp"


//defining motors ports
#define CHASSIS_L1_PORT 1
#define CHASSIS_L2_PORT 2
#define CHASSIS_L3_PORT 3
#define CHASSIS_R1_PORT 4
#define CHASSIS_R2_PORT 5
#define CHASSIS_R3_PORT 6
#define CATA_L_PORT 7
#define CATA_R_PORT 8
#define INTAKE1_PORT 9
#define INTAKE2_PORT 10
#define ROLLER_PORT 11
#define LAUNCHER_PORT 'g'

//defining sensor ports
#define GYRO_PORT 11
#define OPTICAL_PORT 12

//declaring motors
pros::Motor chassis_L1(CHASSIS_L1_PORT);
pros::Motor chassis_L2(CHASSIS_L2_PORT, true);
pros::Motor chassis_L3(CHASSIS_L3_PORT);
pros::Motor chassis_R1(CHASSIS_R1_PORT, true);
pros::Motor chassis_R2(CHASSIS_R2_PORT);
pros::Motor chassis_R3(CHASSIS_R3_PORT,true);
pros::Motor intake_1(INTAKE1_PORT);
pros::Motor intake_2(INTAKE2_PORT,true);
pros::Motor roller(ROLLER_PORT);
pros::Motor catapult_L(CATA_L_PORT);
pros::Motor catapult_R(CATA_R_PORT);

//Declaring sensors and pneumatics
pros::Imu gyro (GYRO_PORT);
pros::ADIDigitalOut launcher(LAUNCHER_PORT);
pros::Controller controller(pros::E_CONTROLLER_MASTER);


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
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
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

	pros::lcd::register_btn1_cb(on_center_button);

	intake_1.set_brake_mode(MOTOR_BRAKE_COAST);
	intake_2.set_brake_mode(MOTOR_BRAKE_COAST);

	
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
		} else {
			chassis_R1.brake();
			chassis_R2.brake();
			chassis_R3.brake();
		}
		if(abs(left) > 5){
			left = left - 5 * (127/122);
			chassis_L1.move(left);
			chassis_L2.move(left);
			chassis_L3.move(left);
		} else {
			chassis_L1.brake();
			chassis_L2.brake();
			chassis_L3.brake();
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
	bool intakeFlag = true;


	
	pros::Task drive(driveTask);
	while (true) {
		if(controller.get_digital(DIGITAL_A) || catapultTarget != 0){
			shoot();
			intakeFlag = false;
		}
	}
}
