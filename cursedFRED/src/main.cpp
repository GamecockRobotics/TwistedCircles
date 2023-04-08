#include "main.h"
#include "pros/adi.hpp"
#include "pros/apix.h"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.h"
#include "pros/serial.hpp"
#include <cstdio>
#include <iostream>
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
enum turnType { right, left };
enum intakeSetting { on, off };
enum opticalType { rightSen, leftSen };

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
pros::ADIDigitalOut endgame(ENDGAME_PORT);
pros::Imu gyro(GYRO_PORT);
pros::ADIDigitalOut rangeSwitch(RANGE_SWITCH_PORT);
pros::ADIDigitalIn SlipGearSensor(LIMIT_PORT);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

double ycord0 = 0;
double xcord0 = 0;
double ycord1 = 0;
double xcord1 = 0;
double theta = 0;

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

int odmentry() {
  int value = 0;

  return value;
}

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

std::string intToString(int number) { return std::to_string(number); }

void sendDataToRaspberryPi(int value) {
  std::string data;
  data = std::to_string(value) + "\n";
  std::cout << data;
}

void sendDataToRaspberryPi(std::string value) { std::cout << value; }

std::string recieveDataToRaspberryPi() {
  std::string data;
  std::cin >> data;
  return data;
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

  pros::c::serctl(SERCTL_DISABLE_COBS, NULL);

  ycord0 = 0;
  xcord0 = 0;
  ycord1 = 0;
  xcord1 = 0;
  theta = 0;

  cataL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cataR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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
  // int i = 0;
  // int j =0;
  // for(i = 65; i<=10000;){
  // 	sendDataToRaspberryPi(j);
  // 	j++;
  //   controller.rumble("----...----");
  // 	pros::delay(20);
  // }

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

    // // String Launcher
    // if (controller.get_digital(DIGITAL_RIGHT) &&
    //     controller.get_digital(DIGITAL_UP) &&
    //     controller.get_digital(DIGITAL_DOWN) &&
    //     controller.get_digital(DIGITAL_LEFT) &&
    //     controller.get_digital(DIGITAL_Y) &&
    //     controller.get_digital(DIGITAL_X) &&
    //     controller.get_digital(DIGITAL_B) &&
    //     controller.get_digital(DIGITAL_A)) {
    //   endgame.set_value(endgameState);
    // }

    // if (controller.get_digital(DIGITAL_X)) {
    //   rangeSwitchToggle(true);
    // }

    pros::delay(20);
  }
}
