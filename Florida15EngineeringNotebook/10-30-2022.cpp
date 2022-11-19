// Include Statements for Pros libraries
#include "main.h"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <string>

// Define the motor ports
#define CHASSIS_L1_PORT 9
#define CHASSIS_L2_PORT 15
#define CHASSIS_L3_PORT 13
#define CHASSIS_R1_PORT 1
#define CHASSIS_R2_PORT 6
#define CHASSIS_R3_PORT 20
#define INTAKE_PORT 5
#define FLYWHEEL_PORT 16
#define TINY_FLY_WHEEL 3
#define ROLLER_PORT 4
#define INDEX_PORT 18
#define GYRO_PORT 19

// The acceptable tolerance for the speed of the flywheel
#define FLYWHEEL_EPSILON 20

/** 
 * Enumerated type to keep track of what state the intake should be in
 * Possible values for the intake state are "in", "out", and "stop"
 */
enum IntakeDirection { in, stop, out };

/**
 * Enumerated Type to determine which direction we are turning in auton
 * Possible values are "left" and "right"
 */
enum TurnType {left, right};

// The initial state of the intake should be in the state "stop"
IntakeDirection intakeState = stop;

/**
 * Variables to keep track of the desired speed of certain motors
 * 
 * The Flywheel motor should have an initial target speed of 0
 * The Indexer motor should have an initial target speed of 0
 */
int flywheelTarget = 0, indexerTarget = 0;

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
pros::Motor intake(INTAKE_PORT, true);
pros::Motor flywheel(FLYWHEEL_PORT, true);
pros::Motor tiny_flywheel(TINY_FLY_WHEEL);
pros::Motor indexer(INDEX_PORT);
pros::Motor roller(ROLLER_PORT);
pros::Imu gyro(GYRO_PORT);

	

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    // Initialize LCD display to help with debugging
	pros::lcd::initialize();

	// Set Chassis encoder values to 0
	pros::c::motor_tare_position(CHASSIS_L1_PORT);
	pros::c::motor_tare_position(CHASSIS_R1_PORT);

	// Set the encoder units to degrees for measuring chassis distance in autonomous
	pros::c::motor_set_encoder_units(CHASSIS_L1_PORT, pros::E_MOTOR_ENCODER_DEGREES);
	pros::c::motor_set_encoder_units(CHASSIS_L2_PORT, pros::E_MOTOR_ENCODER_DEGREES);
	pros::c::motor_set_encoder_units(CHASSIS_L3_PORT, pros::E_MOTOR_ENCODER_DEGREES);
	pros::c::motor_set_encoder_units(CHASSIS_R1_PORT, pros::E_MOTOR_ENCODER_DEGREES);
	pros::c::motor_set_encoder_units(CHASSIS_R2_PORT, pros::E_MOTOR_ENCODER_DEGREES);
	pros::c::motor_set_encoder_units(CHASSIS_R3_PORT, pros::E_MOTOR_ENCODER_DEGREES);
	
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

// Tares all motor positions to help with autonomous
void tareMotors(){
	chassis_l1.tare_position();
	chassis_l2.tare_position();
	chassis_l3.tare_position();
	chassis_r1.tare_position();
	chassis_r2.tare_position();
	chassis_r3.tare_position();
}
/**
 * Drives forward at a specified power a specified distance
 *
 * @param power the speed at which the robot should travel
 * @param distance the distance that the robot should travel in inches
 */
void drive(int power, float distance){
	// Resets motor position to ensure accuracy in autonomous
    tareMotors();
    // Drives specified distance
	chassis_l1.move_absolute(distance*19, power);
	chassis_l2.move_absolute(distance*19, power);
	chassis_l3.move_absolute(distance*19, power);
	chassis_r1.move_absolute(distance*19, power);
	chassis_r2.move_absolute(distance*19, power);
	chassis_r3.move_absolute(distance*19, power);

    // Continue running this loop as long as the motor is not within +-5 units of its goal
	while (!(chassis_l1.get_actual_velocity() == 0)) {
        pros::delay(2);
    }
}

/**
 * Turns the robot in autonomous
 *
 * @param dir the direction that the robot turns
 * @param deg the number of degrees to turn
 */
void turn(TurnType dir, double deg){ 
    // Resets motor position to ensure accuracy in autonomous
	tareMotors(); 
 
    // Calculates the distance to turn based on the physical robot and the number of degrees
	double distance = (dir == left ? -1:1)* 3.8 * deg; 

    // Prints turn distance for debugging properties
	pros::lcd::set_text(1, std::to_string(distance)); 

    // Moves robot calculated distance
	chassis_l1.move_absolute(distance, 100); 
	chassis_l2.move_absolute(distance, 100); 
	chassis_l3.move_absolute(distance, 100); 
	chassis_r1.move_absolute(-distance, 100); 
	chassis_r2.move_absolute(-distance, 100); 
	chassis_r3.move_absolute(-distance, 100); 

    // Continue running this loop as long as the motor is not within +-5 units of its goal 
	while (!((chassis_l1.get_position() < distance+5) && (chassis_l1.get_position() > distance-5))) { 
        pros::delay(2); 
    }
} 

/**
 * Turns the rolling mechanism for the time specified
 *
 * @param time the time in milliseconds to roll
 */
void roll (int time) {
	roller.move(-127);
	pros::delay(time);
	roller.brake();
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
	/**
     * Autonomous
     *
     * Auotnomous moves the robot to the roller and changes the roller to our alliances color
     */
    // Move forward 1 field tile (24 inches) to roller from starting position 
	drive(127, 24);
	pros::delay(100);
    // Turn left 90 degrees to face roller
	turn(left,90);
	pros::delay(100);
    // Drive into roller to force contact between the rolling mechanism and the roller
	drive(-127, 2);
	pros::delay(100);
    // Turn the rollers so that our color is facing up
	roll(250);
	pros::delay(100);
}


/**
 * Checks to ensure Flywheel is at desired speed
 * 
 * @param speed the desired speed to check flywheel speed against
 * @return true if flywheel is greater than dedsired speed otherwise false
 */
bool checkFlywheelSpeed(int speed) {
    /**
     * Calculates flywheel velocity 
     * Flywheel has a 3600 rpm motor, but pros only initializes as a 200rpm motor
     * constant 18 is used to adjust for actual motor cartridge
     */
	return 18*flywheel.get_actual_velocity() > speed;
}


// Shoots a singular disk from the flywheel
void shoot() {
	// Checks if flywheel is at desired speed of 2500 and then turns on indexer
    if (indexerTarget == 0) {
		indexerTarget = 127;
	// Checks is indexer hits physical stopping mechanism, then reverses indexer
	} else if (indexerTarget == 127 && indexer.get_efficiency() < 5) {
		indexerTarget = -127;
	// Checks if indexer hits physical stopping mechanism on return and turns off indexer
	} else if (indexerTarget == -127 && indexer.get_efficiency() < 5) {
		indexerTarget = 0;
	}
    // Sets motor speed to target speed
	indexer = indexerTarget;
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
	// Sets the target value for the left and right side of the chassis respectively
	int leftSpeed = 0, rightSpeed = 0;
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
		leftSpeed = abs(controller.get_analog(ANALOG_LEFT_Y)) > 5 ? controller.get_analog(ANALOG_LEFT_Y) : 0; 
		rightSpeed = abs(controller.get_analog(ANALOG_RIGHT_Y)) > 5 ? controller.get_analog(ANALOG_RIGHT_Y) : 0; 

        // Set Chassis Motors to target speed
		chassis_l1.move(leftSpeed);
		chassis_l2.move(leftSpeed);
		chassis_l3.move(leftSpeed);
		chassis_r1.move(rightSpeed);
		chassis_r2.move(rightSpeed);
		chassis_r3.move(rightSpeed);

		/**
         * Intake and Indexer Code
         *
         * On Button A press shoot the disk with the indexer
         * On Button R1 presses toggle intake between intake and pause
         */ 
		if (controller.get_digital(DIGITAL_A) || indexerTarget != 0){ 
			shoot(); 
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


		/**
         * Rollers
         * Sets roller speed based on user control
         * 
         * On Button Up the rollers spin forward
         * On button Down the rollers spin in reverse
         */
		if (controller.get_digital(DIGITAL_UP)) {
			roller = 127;
		} else if (controller.get_digital(DIGITAL_DOWN)) {
			roller = -127;
		} else {
			roller = 0;
		}

        /**
          * Used for debugging
          *
          * displays flywheel speed on line 2
          * displays indexer speed on line 4
          * displays the indexer efficency to determine when it hits the physical stop on line 6
          */
		pros::lcd::set_text(2, "flywheel: " + std::to_string(flywheel.get_actual_velocity()*18));
		pros::lcd::set_text(4, "indexer target: " + std::to_string(indexerTarget));
		pros::lcd::set_text(6, "indexer efficiency: " + std::to_string(indexer.get_efficiency()));

		// Set flywheel and indexing flex wheel to maximum speed
		flywheel.move(127);
		tiny_flywheel.move(127);

        // Pause to allow other processes to run
		pros::delay(20);
	}
}
