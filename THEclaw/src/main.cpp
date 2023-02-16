#include "main.h"


#define INTAKEL_PORT 6
#define INTAKER_PORT 5
#define CATAL_PORT 19
#define CATAR_PORT 20
#define GYRO_PORT 18
#define LIMIT_PORT 'g'
#define CHASSIS_L1_PORT 7 //top front motor
#define CHASSIS_L2_PORT 8 //Top back motor
#define CHASSIS_L3_PORT 9 // bottom front motor
#define CHASSIS_L4_PORT 10 // bottom back motor
#define CHASSIS_R1_PORT 1 // top front motor
#define CHASSIS_R2_PORT 2 // top back motor
#define CHASSIS_R3_PORT 3 // bottom front motor
#define CHASSIS_R4_PORT 4 // bottom back motor 


pros::Motor chassis_r1(CHASSIS_R1_PORT, true);
pros::Motor chassis_r2(CHASSIS_R2_PORT, true);
pros::Motor chassis_r3(CHASSIS_R3_PORT);
pros::Motor chassis_r4(CHASSIS_R4_PORT);
pros::Motor chassis_l1(CHASSIS_L1_PORT);
pros::Motor chassis_l2(CHASSIS_L2_PORT);
pros::Motor chassis_l3(CHASSIS_L3_PORT, true);
pros::Motor chassis_l4(CHASSIS_L4_PORT, true);

pros::Motor cataL(CATAL_PORT);
pros::Motor cataR(CATAR_PORT,true);
pros::Motor intake_1(INTAKEL_PORT,true);
pros::Motor intake_2(INTAKER_PORT);

pros::ADIDigitalIn SlipGearSensor (LIMIT_PORT);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
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

	cataL.set_brake_mode(MOTOR_BRAKE_HOLD);
    cataR.set_brake_mode(MOTOR_BRAKE_HOLD);

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

int driveTask(){
	while(true){
		//Base Tank Controls
		int left = controller.get_analog(ANALOG_LEFT_Y);
		int right = controller.get_analog(ANALOG_RIGHT_Y);
		if(abs(right) > 5){
			right = right -5 * (127/122);
			chassis_r1.move(right);
			chassis_r2.move(right);
			chassis_r3.move(right);
			chassis_r4.move(right);
		} else {
			chassis_r1.brake();
			chassis_r2.brake();
			chassis_r3.brake();
			chassis_r4.brake();
		}
		if(abs(left) > 5){
			left = left - 5 * (127/122);
			chassis_l1.move(left);
			chassis_l2.move(left);
			chassis_l3.move(left);
			chassis_l4.move(left);
		} else {
			chassis_l1.brake();
			chassis_l2.brake();
			chassis_l3.brake();
			chassis_l4.brake();
		}

	
	}
	
	return 0;
}
void opcontrol() {
	int cataFlag = 0;
	pros::Task drive(driveTask);
	while (true) {

		//Catapult pull back on Button Sensor
        if (!SlipGearSensor.get_value()) {
            cataFlag = 1;
            cataL.move_velocity(600);
            cataR.move_velocity(600);
            pros::lcd::set_text(3, "up" + std::to_string(SlipGearSensor.get_value()));
        } else if (SlipGearSensor.get_value()) {
            cataFlag = 0;
            //intakeLock = 0;
            cataL.brake();
            cataR.brake();
            pros::lcd::set_text(3, "down" + std::to_string(SlipGearSensor.get_value()));
        }

		 if(controller.get_digital(DIGITAL_R1) && cataFlag == 0 ){
            cataFlag = 0;
            //intakeLock = 1;
            cataL.move_velocity(600);
            cataR.move_velocity(600);
         }

		//Intake
        if(controller.get_digital(DIGITAL_L1)){
            intake_1.move(-127);
            intake_2.move(-127);
        }else if (controller.get_digital(DIGITAL_L2)){
        	intake_1.move(127);
            intake_2.move(127);
        } else {
            intake_1.brake();
            intake_2.brake();
        }
	}
}
