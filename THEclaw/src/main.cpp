#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/optical.hpp"
#include <string>

#define INTAKEL_PORT 6
#define INTAKER_PORT 5
#define CATAL_PORT 19
#define CATAR_PORT 20
#define GYRO_PORT 18
#define LIMIT_PORT 'g'
#define VISION1_PORT 11
#define VISION2_PORT 12
#define RANGE_SWITCH_PORT 'h' // These are pistons
#define ENDGAME_PORT 'f'      // These are pistons
#define CHASSIS_L1_PORT 7     // top front motor
#define CHASSIS_L2_PORT 8     // Top back motor
#define CHASSIS_L3_PORT 9     // bottom front motor
#define CHASSIS_L4_PORT 10    // bottom back motor
#define CHASSIS_R1_PORT 1     // top front motor
#define CHASSIS_R2_PORT 2     // top back motor
#define CHASSIS_R3_PORT 3     // bottom front motor
#define CHASSIS_R4_PORT 4     // bottom back motor

enum color { red, blue };
enum opticalType { rightSen, leftSen };
enum turnType { right, left };
enum intakeSetting { on, off };
// declaring motors
pros::Motor chassis_r1(CHASSIS_R1_PORT, true);
pros::Motor chassis_r2(CHASSIS_R2_PORT, true);
pros::Motor chassis_r3(CHASSIS_R3_PORT);
pros::Motor chassis_r4(CHASSIS_R4_PORT);
pros::Motor chassis_l1(CHASSIS_L1_PORT);
pros::Motor chassis_l2(CHASSIS_L2_PORT);
pros::Motor chassis_l3(CHASSIS_L3_PORT, true);
pros::Motor chassis_l4(CHASSIS_L4_PORT, true);

pros::Motor cataL(CATAL_PORT);
pros::Motor cataR(CATAR_PORT, true);
pros::Motor intake_1(INTAKEL_PORT, true);
pros::Motor intake_2(INTAKER_PORT);

pros::ADIDigitalIn SlipGearSensor(LIMIT_PORT);
pros::ADIDigitalOut rangeSwitch(RANGE_SWITCH_PORT);
pros::ADIDigitalOut endgame(ENDGAME_PORT);
pros::Optical vision1(VISION1_PORT);
pros::Optical vision2(VISION2_PORT);
pros::Imu gyro(GYRO_PORT);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

bool endgameState = true;

void tareMotors() {
  // Resets all drive train motor positions to 0
  chassis_l1.tare_position();
  chassis_l2.tare_position();
  chassis_l3.tare_position();
  chassis_l4.tare_position();
  chassis_r1.tare_position();
  chassis_r2.tare_position();
  chassis_r3.tare_position();
  chassis_r4.tare_position();
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

  cataL.set_brake_mode(MOTOR_BRAKE_HOLD);
  cataR.set_brake_mode(MOTOR_BRAKE_HOLD);

  intake_1.set_brake_mode(MOTOR_BRAKE_COAST);
  intake_2.set_brake_mode(MOTOR_BRAKE_COAST);

  endgame.set_value(false);
  tareMotors();
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

void rangeSwitchToggle(bool rangeToggle) { rangeSwitch.set_value(rangeToggle); }

void intakeSetting(intakeSetting setting) {
  // Turns intake either on or off
  if (setting == off) {
    intake_1 = 0;
    intake_2 = 0;
  } else if (setting == on) {
    intake_1 = -112;
    intake_2 = -112;
  }
}

color get_color(double hue) {
  // Returns red or blue based off of the hue value seen by the vision sensor
  return (hue < 20) ? red : blue;
}

void runRoller(int speed = 40, opticalType type = rightSen) {
  // Scores roller if the roller is at competition start position
  // Turns on flashlight
  type == rightSen ? vision1.set_led_pwm(100) : vision2.set_led_pwm(100);
  pros::lcd::set_text(4, std::to_string(vision1.get_hue()));

  // Slowly backs into roller
  chassis_l1 = -20;
  chassis_l2 = -20;
  chassis_l3 = -20;
  chassis_l4 = -20;
  chassis_r1 = -20;
  chassis_r2 = -20;
  chassis_r3 = -20;
  chassis_r4 = -20;

  int counter = 0;
  bool startColor;
  startColor = type == rightSen ? get_color(vision1.get_hue())
                                : get_color(vision2.get_hue());

  // Spins roller until it sees a different color than what it started at

  while (startColor == (type == rightSen ? get_color(vision1.get_hue())
                                         : get_color(vision2.get_hue())) &&
         counter < 200) {
    intake_1.move(speed);
    intake_2.move(speed);
    counter++;
    pros::delay(10);
  }

  // Stops the intake, robot movement, and turns off flashlight
  intake_1.brake();
  intake_2.brake();
  chassis_l1 = 0;
  chassis_l2 = 0;
  chassis_l3 = 0;
  chassis_l4 = 0;
  chassis_r1 = 0;
  chassis_r2 = 0;
  chassis_r3 = 0;
  chassis_r4 = 0;
  type == rightSen ? vision1.set_led_pwm(0) : vision2.set_led_pwm(0);
}

void drive(int power, float distance) {
  // Resets motor position to ensure accuracy in autonomous
  tareMotors();
  // Drives specified distance
  // Multiplies distance by the conversion factor for inches
  chassis_l1.move_absolute(distance * 13.37 * 2.5 - 5, power);
  chassis_l2.move_absolute(distance * 13.37 * 2.5 - 5, power);
  chassis_l3.move_absolute(distance * 13.37 * 2.5 - 5, power);
  chassis_l4.move_absolute(distance * 13.37 * 2.5 - 5, power);
  chassis_r1.move_absolute(distance * 13.37 * 2.5, power);
  chassis_r2.move_absolute(distance * 13.37 * 2.5, power);
  chassis_r3.move_absolute(distance * 13.37 * 2.5, power);
  chassis_r4.move_absolute(distance * 13.37 * 2.5, power);
  pros::lcd::set_text(3, "driving");
  // Waits a second to allow motors to build speed
  pros::delay(1000);
  // Continue running this loop as long as the motor is moving, locking the
  // program in the function
  while (!(chassis_l1.get_actual_velocity() == 0)) {
    pros::delay(10);
    // Prints motor voltage values
    pros::lcd::set_text(
        4,
        std::to_string((chassis_l1.get_voltage() + chassis_l2.get_voltage() +
                        chassis_l3.get_voltage() + chassis_l4.get_voltage()) /
                       4));
    pros::lcd::set_text(
        5,
        std::to_string((chassis_r1.get_voltage() + chassis_r2.get_voltage() +
                        chassis_r3.get_voltage() + chassis_r4.get_voltage()) /
                       4));
  }
  pros::lcd::set_text(3, "done driving");
}

void turn(int32_t deg, double precision) {
  // PID loop to control turning
  /*
  Turntype is legacy code that used to determine if the robot was turning left
  or right, this code was bugged to where left did not work so we defaulted to
  turning right, and using negative values to turn left
  */
  turnType dir = right;
  tareMotors();
  // Gets initial gyro position
  float initialValue = gyro.get_rotation();
  // Sets error to the degree that we are turning to, degree is based off
  // starting rotation at the beginning of the match, not current rotation
  float error = deg - initialValue;
  float prevError = error;
  float totalError = 0;
  const float threshold = precision;
  const float kp = 1.25; // was 1.4
  const float ki = 0.9;  // was .3
  const float kd = .78;  // was .78
  std::string first = std::to_string(gyro.get_rotation());
  pros::lcd::set_text(7, "");
  pros::lcd::set_text(4, "Gyro Value: " + first);
  // Loops PID until the error is within the threshold for 2 cycles
  while (fabs(error) > threshold || fabs(prevError) > threshold) {
    int speed =
        (kp * error + kd * (error - prevError) + ki * totalError) * 9 / 10;
    // Move motors
    chassis_l1.move(dir == left ? -speed : speed);
    chassis_l2.move(dir == left ? -speed : speed);
    chassis_l3.move(dir == left ? -speed : speed);
    chassis_l4.move(dir == left ? -speed : speed);
    chassis_r1.move(dir == right ? -speed : speed);
    chassis_r2.move(dir == right ? -speed : speed);
    chassis_r3.move(dir == right ? -speed : speed);
    chassis_r4.move(dir == right ? -speed : speed);
    pros::delay(200);
    prevError = error;
    std::string second = std::to_string((float)gyro.get_rotation());
    pros::lcd::set_text(5, "Gyro Value In while: " + second);
    // Updates new error value
    error = deg - (gyro.get_rotation());
    std::string errorInWhile = std::to_string(error);
    pros::lcd::set_text(6, "Error Value: " + errorInWhile);
    // Increments total error while error is less than 20, allows for fine turns
    totalError += (fabs(error) < 20 ? error : 0); // was 10
    // Flips direction of total error and reduces it if the goal is overshot
    totalError *= (std::signbit(error) == std::signbit(prevError)) ? 1 : -3 / 4;
  }
  pros::lcd::set_text(7, "I'm out of PID");
  chassis_r1.brake();
  chassis_r2.brake();
  chassis_r3.brake();
  chassis_r4.brake();
  chassis_l1.brake();
  chassis_l2.brake();
  chassis_l3.brake();
  chassis_l4.brake();
}

void shoot() {
  cataL.move_velocity(600);
  cataR.move_velocity(600);
  pros::delay(600);
  
  while (!SlipGearSensor.get_value()) {
    cataL.move_velocity(200);
    cataR.move_velocity(200);
    pros::delay(10);
  }

  cataL.brake();
  cataR.brake();
}

void skills_roller_temp() {
  // Turn on flashlight
	vision2.set_led_pwm(100);
	
	// Drive forward into roller
    chassis_l1 = -10;
    chassis_l2 = -10;
    chassis_l3 = -10;
    chassis_l4 = -10;
    chassis_r1 = -10;
    chassis_r2 = -10;
    chassis_r3 = -10;
    chassis_r4 = -10;

	pros::delay(400);

	intake_1.move(-40);
  intake_2.move(-40);
	pros::delay(200);
	intake_1.move(127);
  intake_2.move(127);
	pros::delay(200);

	while (get_color(vision2.get_hue()) != red) {
	intake_1.move(50);
  intake_2.move(50);
		// delay to allow other tasks to run
		pros::delay(10);
	}
	while (get_color(vision2.get_hue()) == red) {
	intake_1.move(50);
  intake_2.move(50);
		// delay to allow other tasks to run
		pros::delay(10);
	}
	while (get_color(vision2.get_hue()) != red) {
	intake_1.move(-30);
  intake_2.move(-30);
		// delay to allow other tasks to run
		pros::delay(10);
	}

	// Stops the intake, robot movement, and turns off flashlight
	intake_1.brake();
  intake_2.brake();
	
	// brake chassis
    chassis_l1.brake();
    chassis_l2.brake();
    chassis_l3.brake();
    chassis_l4.brake();
    chassis_r1.brake();
    chassis_r2.brake();
    chassis_r3.brake();
    chassis_r4.brake();

	// turn off flashlight
	vision2.set_led_pwm(0);
}

void good_skills_roller(pros::Optical vision = vision2) {
    vision.set_led_pwm(100);

    chassis_l1 = -5;
    chassis_l2 = -5;
    chassis_l3 = -5;
    chassis_l4 = -5;
    chassis_r1 = -5;
    chassis_r2 = -5;
    chassis_r3 = -5;
    chassis_r4 = -5;

    intake_1.move(-40);
    intake_2.move(-40);
    pros::delay(300);
    intake_1.move(127);
    intake_2.move(127);
    pros::delay(800);
    double speed = 0;
    while(get_color(vision.get_hue()) == blue) {
      speed += .2;
      intake_1.move(speed);
      intake_2.move(speed);
      pros::delay(10);
    }
    while(get_color(vision.get_hue()) == red) {
      speed += .2;
      intake_1.move(speed);
      intake_2.move(speed);
      pros::delay(10);
    }
    double rev_speed = 0;
    while(get_color(vision.get_hue()) == blue) {
      rev_speed -= .2;
      intake_1.move(rev_speed);
      intake_2.move(rev_speed);
      pros::delay(10);
    }
    intake_1.brake();
    intake_2.brake();
    chassis_l1.brake();
    chassis_l2.brake();
    chassis_l3.brake();
    chassis_l4.brake();
    chassis_r1.brake();
    chassis_r2.brake();
    chassis_r3.brake();
    chassis_r4.brake();
    vision.set_led_pwm(0);
}

void skills_roller(int speed = -127, pros::Optical vision = vision2) {
    // Turns on flashlight and slowly backs into roller
    vision.set_led_pwm(100);
    pros::lcd::set_text(4, std::to_string(vision.get_hue()));
    chassis_l1 = -5;
    chassis_l2 = -5;
    chassis_l3 = -5;
    chassis_l4 = -5;
    chassis_r1 = -5;
    chassis_r2 = -5;
    chassis_r3 = -5;
    chassis_r4 = -5;
    int counter = 0;

    // Puts roller into a state that wont get stuck on screw
    intake_1.move(40);
    intake_2.move(40);
    pros::delay(500);
    
    // 2 while loops spin until color changes twice
    while(get_color(vision.get_hue()) == red && counter < 200) {
        intake_1.move(speed);
        intake_2.move(speed);
        counter++;
        pros::delay(10);
    }
    
    counter = 0;
    speed += 70;

    while(get_color(vision.get_hue()) != red && counter < 200) {
        intake_1.move(speed);
        intake_2.move(speed);
        counter++;
        pros::delay(10);
    }
    counter = 0;
    speed += 20;
    while(get_color(vision.get_hue()) == red) {
      intake_1.move(-speed);
      intake_2.move(-speed);
      counter++;
      pros::delay(10);
    }

    // Stops the intake, robot movement, and turns off flashlight
    intake_1.brake();
    intake_2.brake();
    chassis_l1.brake();
    chassis_l2.brake();
    chassis_l3.brake();
    chassis_l4.brake();
    chassis_r1.brake();
    chassis_r2.brake();
    chassis_r3.brake();
    chassis_r4.brake();
    vision.set_led_pwm(0);
}

void new_skills_roller() {
  intake_1.move(-30);
  intake_2.move(-30);
  pros::delay(200);
  intake_1.move_relative(900, 200);
  intake_2.move_relative(900, 200);
  pros::delay(1000);
}

void autonomous() {
  int cataFlag;
  // For testing turn
  while (gyro.is_calibrating()) {
    pros::delay(10);
  };

  while (!SlipGearSensor.get_value()) {
    cataL.move_velocity(400);
    cataR.move_velocity(400);
    pros::lcd::set_text(3, "up" + std::to_string(SlipGearSensor.get_value()));
  }

  cataL.brake();
  cataR.brake();

  // Competition auton
  // shoot();
  // pros::delay(200);
  // // Get First roller
  // drive(20, -20);
  // pros::delay(200);
  // drive(40, 2);
  // pros::delay(200);
  // runRoller();
  // // Drive to second roller
  // pros::delay(200);
  // gyro.set_rotation(0);
  // drive(20, 5);
  // pros::delay(200);
  // turn(-90, 1);
  // intakeSetting(on);
  // pros::delay(200);
  // drive(50, -50);
  // pros::delay(50);
  // turn(-90, 1);
  // pros::delay(50);
  // drive(40, -80);
  // pros::delay(200);
  // drive(30, 7);
  // pros::delay(200);
  // turn(-180, 6);
  // pros::delay(200);
  // drive(20, 10);
  // gyro.set_rotation(-180);
  // pros::delay(400);
  // drive(50, -70);
  // pros::delay(400);
  // turn(-180, 1);
  // pros::delay(400);
  // drive(50, -55);
  // pros::delay(200);
  // intakeSetting(off);
  // // Get second roller
  // turn(-90, 1);
  // pros::delay(200);
  // drive(40, -25);
  // pros::delay(200);
  // drive(40, 1);
  // pros::delay(200);
  // runRoller(45, leftSen);
  // pros::delay(200);
  // rangeSwitchToggle(true);

  // Skills Auton

  // Shoot preload
  gyro.set_rotation(0);
  shoot();
  // Get more disks
  // turn(-55, 1);
  // pros::delay(200);
  // intakeSetting(on);
  // drive(40, -35);
  // pros::delay(200);
  // drive(80, -2);
  // pros::delay(200);
  // turn(-135, 1);
  // drive(40, -22);
  // pros::delay(200);
  // drive(80, -2);
  // pros::delay(200);
  // drive(40, -22);
  // pros::delay(200);
  // drive(80, -2);
  // pros::delay(200);
  // turn(-45, 1);
  // shoot();
  // // Head to goal
  // turn(0, 1);
  // pros::delay(200);
  // drive(40, 50);
  // pros::delay(200);
  // turn(-10, 1);
  // pros::delay(200);
  // //Shoot
  // shoot();
  // pros::delay(200);
  // Go to roller
  turn(85, 1);
  pros::delay(200);
  drive(30, -15);
  pros::delay(200);
  gyro.set_rotation(90);
  drive(30, 7);
  pros::delay(200);
  turn(0, 0.5);
  pros::delay(200);
  drive(40, -48);
  pros::delay(200);
  // Get First Roller
  turn(90, 1);
  pros::delay(200);
  drive(20, -20);
  // pros::delay(200);
  // drive(40, 2);
  pros::delay(200);
  new_skills_roller();
  // Move to second roller
  gyro.set_rotation(90);
  drive(40, 5);
  pros::delay(200);
  turn(-45, 1);
  pros::delay(200);
  intakeSetting(on);
  drive(30, -16);
  pros::delay(200);
  drive(80, -4);
  turn(0, 1);
  intakeSetting(off);
  // Get Second roller
  drive(20, -20);
  pros::delay(200);
  // drive(40, 2);
  // pros::delay(200);
  new_skills_roller();
  // Move to final postion
  drive(30, 20);
  pros::delay(200);
  turn(45, 1);
  pros::delay(200);
  drive(30, -7);
  // Wait for endgame
  // pros::delay(20000);
  endgame.set_value(endgameState);
  // // Test Roller
  // drive(40, -2);
  // pros::delay(200);
  // drive(40, .5);
  // pros::delay(200);
  // good_skills_roller();
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

int driveTask() {
  while (true) {
    // Base Tank Controls
    int left = controller.get_analog(ANALOG_LEFT_Y);
    int right = controller.get_analog(ANALOG_RIGHT_Y);
    if (abs(right) > 5) {
      right = right - 5 * (127 / 122);
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
    if (abs(left) > 5) {
      left = left - 5 * (127 / 122);
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

    // Catapult pull back on Button Sensor
    if (!SlipGearSensor.get_value()) {
      cataFlag = 1;
      cataL.move_velocity(600);
      cataR.move_velocity(600);
      pros::lcd::set_text(3, "up" + std::to_string(SlipGearSensor.get_value()));
    } else if (SlipGearSensor.get_value()) {
      cataFlag = 0;
      // intakeLock = 0;
      cataL.brake();
      cataR.brake();
      pros::lcd::set_text(3,
                          "down" + std::to_string(SlipGearSensor.get_value()));
    }

    if (controller.get_digital(DIGITAL_R1) && cataFlag == 0) {
      cataFlag = 0;
      // intakeLock = 1;
      cataL.move_velocity(600);
      cataR.move_velocity(600);
    }
    if (controller.get_digital(DIGITAL_R2)) {
      cataL.move_velocity(-600);
      cataR.move_velocity(-600);
    }

    // Intake
    if (controller.get_digital(DIGITAL_L1)) {
      intake_1.move(-110);
      intake_2.move(-110);
    } else if (controller.get_digital(DIGITAL_L2)) {
      intake_1.move(127);
      intake_2.move(127);
    } else {
      intake_1.brake();
      intake_2.brake();
    }

    if (controller.get_digital(DIGITAL_X)) {
      vision2.set_led_pwm(100);
      pros::lcd::set_text(4, std::to_string(vision2.get_hue()));
    }

    // String Launcher

    if (controller.get_digital(DIGITAL_RIGHT) &&
        controller.get_digital(DIGITAL_UP) &&
        controller.get_digital(DIGITAL_DOWN) &&
        controller.get_digital(DIGITAL_LEFT) &&
        controller.get_digital(DIGITAL_Y) &&
        controller.get_digital(DIGITAL_X) &&
        controller.get_digital(DIGITAL_B) &&
        controller.get_digital(DIGITAL_A)) {
      endgame.set_value(endgameState);
    }

    // if (controller.get_digital(DIGITAL_X)) {
    //   rangeSwitchToggle(true);
    // }
  }
}
