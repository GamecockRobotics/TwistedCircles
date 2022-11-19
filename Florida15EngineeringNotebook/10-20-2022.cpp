// Include Statements for Pros libraries
#include "main.h" 
#include "pros/misc.h" 
#include "pros/motors.h" 
#include "pros/rtos.hpp" 
#include <string> 
 
// Define the motor ports
#define CHASSIS_L1_PORT 9 
#define CHASSIS_L2_PORT 11 
#define CHASSIS_L3_PORT 15 
#define CHASSIS_R1_PORT 1 
#define CHASSIS_R2_PORT 6 
#define CHASSIS_R3_PORT 10 
#define INTAKE_PORT 5 
#define FLYWHEEL_PORT 16 

// The acceptable tolerance for the speed of the flywheel
#define FLYWHEEL_EPSILON 20 

/** 
 * Enumerated type to keep track of what state the intake should be in
 * Possible values for the intake state are "in", "out", and "stop"
 *
 * The initial state of the intake should be in the state "stop"
 */
enum IntakeDirection { in, stop, out }; 
IntakeDirection intakeState = stop; 

/** 
 * Enumerated type to keep track of what speed the flywheel should be
 * Possible values for the flywheel speed are "low", "med", and "high"
 *
 * The initial speed of the flywheel should be at the lowest speed
 */
enum FlywheelSpeed { low, med, high }; 
FlywheelSpeed flywheelSpeed = low; 

/**
 * Variables to keep track of the desired speed of certain motors
 * 
 * The motors on the right side of the chassis should have an initial target speed of 0
 * The motors on the left side of the chassis should have an initial target speed of 0
 * The Flywheel motor should have an initial target speed of 0
 */
int rightTarget = 0, leftTarget = 0, flywheelTarget = 0; 
 
 
/**
 * Create instances of the motors and controller
 *
 * The left side of the chassis is reversed
 * The flywheel is reversed
 */
pros::Controller controller(pros::E_CONTROLLER_MASTER); 
pros::Motor chassis_r1(CHASSIS_R1_PORT); 
pros::Motor chassis_r2(CHASSIS_R2_PORT); 
pros::Motor chassis_r3(CHASSIS_R3_PORT); 
pros::Motor chassis_l1(CHASSIS_L1_PORT, true); 
pros::Motor chassis_l2(CHASSIS_L2_PORT, true); 
pros::Motor chassis_l3(CHASSIS_L3_PORT, true); 
pros::Motor intake(INTAKE_PORT); 
pros::Motor flywheel(FLYWHEEL_PORT, true); 
	
 
/** 
 * Runs initialization code. This occurs as soon as the program is started. 
 * 
 * All other competition modes are blocked by initialize; it is recommended 
 * to keep execution time for this mode under a few seconds. 
 */ 
void initialize() { 
    
    // Set Chassis encoder values to 0
	pros::c::motor_tare_position(CHASSIS_L1_PORT); 
	pros::c::motor_tare_position(CHASSIS_R1_PORT); 
	
    // Set the encoder units to degrees for measuring chassis distance in autonomous
	pros::c::motor_set_encoder_units(CHASSIS_L1_PORT, pros::E_MOTOR_ENCODER_DEGREES); 
	pros::c::motor_set_encoder_units(CHASSIS_R1_PORT, pros::E_MOTOR_ENCODER_DEGREES); 
	

    /**
     * Set the brake mode for certain motors
     *
     * Flywheel is set to hold to prevent it from spinning when not desired
     * Intake is set to coast to prevent intake from siezing up while trying to move a disk
     */
	flywheel.set_brake_mode(MOTOR_BRAKE_HOLD); 
	intake.set_brake_mode(MOTOR_BRAKE_COAST); 
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
 * Sets the intake state 
 * A launches the disk 
 * R1 toggles the intake between on and off 
 * 
 * @param flag to ensure toggle exactly once per button press 
 * @return new flag value 
 */ 
bool toggleIntake(bool flag) {  
    // Incomplete function: returns true as placeholder until code is finished
	return true; 
} 


// Sets the flywheel speed based on the inputs of the Controller
void driverFlywheel() { 
    /**
     * Will set speed to high, med, and low based on input of Controller's B, A, and X buttons.
     * 
     * Waiting on user preference to determine which button sets the flywheel
     * Incomplete: Currently just sets flywheel to 100 as placehoder
     */
	if (controller.get_digital(DIGITAL_B)) { 
		 
	} else if (controller.get_digital(DIGITAL_A)) { 
 
	} else if (controller.get_digital(DIGITAL_X)) { 
 
	}

	flywheel.move_velocity(100);

    /** 
     * Statement to be used to ensure that flywheel gets to the approximate speed
     * within the acceptable tolerance
     */
	if (fabs(flywheelTarget-flywheel.get_actual_velocity()) < FLYWHEEL_EPSILON) { 
 
	} 
} 
 
/** 
 * Task for controlling chassis during driver control 
 * Operates asynchronously to main loop 
 * Uses PID to optimize acceleration and deceleration of the chassis
 */ 
int driveTask() { 
	double kp = 0.6, kd = 0.24, ki = 0.0; 
  	double rightP = 0, leftP = 0, rightD = 0, leftD = 0, rightI = 0, leftI = 0; 
  	double prevRightSpeed = 0, prevLeftSpeed = 0, rightSpeed = 0, leftSpeed = 0; 
  	while (true) { 
		rightI += ki*rightP/kp; 
		rightP = kp*(rightTarget-rightSpeed); 
		rightD = kd*(prevRightSpeed-rightSpeed); 
		prevRightSpeed = rightSpeed; 
		leftI += ki*rightP/kp; 
		leftP = kp*(leftTarget-leftSpeed); 
		leftD = kd*(prevLeftSpeed-leftSpeed); 
		prevLeftSpeed = leftSpeed; 
		rightSpeed += rightSpeed == rightTarget ? 0 : rightP + rightI + rightD ; 
		leftSpeed += (leftSpeed == leftTarget) ? 0 : leftP + leftI + leftD; 
		chassis_l1.move(leftSpeed); 
		chassis_l2.move(leftSpeed); 
		chassis_l3.move(leftSpeed); 
		chassis_r1.move(rightSpeed); 
		chassis_r2.move(rightSpeed); 
		chassis_r3.move(rightSpeed); 
		pros::delay(10); 
	} 
	return 0;
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
    // Boolean flag to ensure that the intake toggles no more than once on button press
	bool intakeFlag = true; 
    // Starts the Asynchronous PID loop for the drive control
	pros::Task drive(driveTask);
    // The main loop for user control
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
         * Sets left side of Chassiss to left joystick value
         * Sets right side of chassis to right joystick value
         * 
         * Joystick has a small deadzone to prevent accidental movements when 
         * the joysticks are not perfectly centered
         */ 
		leftTarget = abs(controller.get_analog(ANALOG_LEFT_Y)) > 5 ? controller.get_analog(ANALOG_LEFT_Y) : 0; 
		rightTarget = abs(controller.get_analog(ANALOG_RIGHT_Y)) > 5 ? controller.get_analog(ANALOG_RIGHT_Y) : 0; 
 

		/**
         * Intake Code
         *
         * On Button A press the intake reverses to feed disks into the flywheel
         * On Button R1 presses toggle intake between intake and pause
         */
		if (controller.get_digital(DIGITAL_A)){ 
			intakeState = out; 
            // Resets intake flag on shoot
			intakeFlag = true; 
		} else if (controller.get_digital(DIGITAL_R1)){ 
			intakeState = (intakeState == in) == intakeFlag ? stop : in; 
            // Sets intake flag to prevent toggle on next iteration on loop
			intakeFlag = false; 
		} else { 
            // Resets intake flag on button release
			intakeFlag = true; 
		} 
 
        // Sets intake speed based on value of intake state
		if (intakeState == in) { 
			intake.move(127); 
		} else if (intakeState == out) { 
			intake.move(-127); 
		} else { 
			intake.brake(); 
		} 
 
        // Sets flywheel speed to 100
		flywheel.move(100); 
 
        // Pause to allow other processes to run
		pros::delay(20); 
	} 
} 
 