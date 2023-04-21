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

#define INTAKEL_PORT 15
#define INTAKER_PORT 5
#define CATAL_PORT 16
#define CATAR_PORT 6
#define GYRO_PORT 12
#define LIMIT_PORT 'g'
#define VISION1_PORT 14
#define VISION2_PORT 
#define RANGE_SWITCH_PORT 'f' // These are pistons
#define ENDGAME_PORT 'a'      // These are pistons
#define CHASSIS_L1_PORT 18     // top front motor
#define CHASSIS_L2_PORT 17     // Top back motor
#define CHASSIS_L3_PORT 20     // bottom front motor
#define CHASSIS_L4_PORT 19    // bottom back motor
#define CHASSIS_R1_PORT 8     // top front motor
#define CHASSIS_R2_PORT 7     // top back motor
#define CHASSIS_R3_PORT 10     // bottom front motor
#define CHASSIS_R4_PORT 9     // bottom back motor
#define ROLLER_PORT 11
#define TRACKING_SIDE_PORT 2
#define TRACKING_FORWARD_PORT 1


enum color { red, blue };

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

pros::Motor cataL(CATAL_PORT, true);
pros::Motor cataR(CATAR_PORT);
pros::Motor intake_1(INTAKEL_PORT);
pros::Motor intake_2(INTAKER_PORT,true);
pros::Motor roller(ROLLER_PORT);
pros::ADIDigitalOut endgame(ENDGAME_PORT);
pros::Imu gyro(GYRO_PORT);
pros::ADIDigitalOut rangeSwitch(RANGE_SWITCH_PORT);
pros::ADIDigitalIn SlipGearSensor(LIMIT_PORT);

pros::Optical vision(VISION1_PORT);

pros::Rotation tracking_side(TRACKING_SIDE_PORT);
pros::Rotation tracking_forward(TRACKING_FORWARD_PORT, true);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

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

// The angle the robot is facing
static constexpr double start_theta = 225; 
// The x coordinate of our alliance goal in millimeters
static constexpr int goal_x = 457;
// The y coordinate of our alliance goal in millimeters
static constexpr int goal_y = 457;


// Varaiables to keep track of the Location of the Robot
double x_loc = 3263, y_loc = 1420;
// Variable to keep track of the Orientation of the Robot
double theta;

// Variables to store the target speed for each side of the chassis
int left_target = 0, right_target = 0;

bool endgameState = true, rangestate = true;

bool cataFlag = false;

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

void chassisL(int speed){
  chassis_l1 = speed;
  chassis_l2 = speed;
  chassis_l3 = speed;
  chassis_l4 = speed;
}

void chassisR(int speed){
  chassis_r1 = speed;
  chassis_r2 = speed;
  chassis_r3 = speed;
  chassis_r4 = speed;
}
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

		pros::lcd::set_text(0, 
		"x: " + std::to_string((int)(x_loc*mm_to_inch)) + 
		"    y: " + std::to_string((int)(y_loc*mm_to_inch)) + 
		"    theta: " + std::to_string((int)theta));

		
	}
	return 0;
}

int cataReset(){
  while (true) {
      pros::lcd::set_text(1, "cataFlag: " + std::to_string((cataFlag)));
      if (cataFlag) {
        cataL.move_velocity(600);
        cataR.move_velocity(600);
        if (!SlipGearSensor.get_value())
          cataFlag = false;
      } else if(!SlipGearSensor.get_value()){
        cataL.move_velocity(600);
        cataR.move_velocity(600);
      }else{
        cataL.brake();
        cataR.brake();
      }
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
	return sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1));
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

void shoot(){
    while(!SlipGearSensor.get_value()){
      pros::delay(10);
    }
    cataFlag = true;

}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  //pros::lcd::set_text(1, "Hello PROS User!");

  pros::c::serctl(SERCTL_DISABLE_COBS, NULL);

  pros::delay(3000);

  rangeSwitchToggle(rangestate);

  cataL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cataR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  pros::Task odometry_task(odometry);
  pros::Task cata_task(cataReset);
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
void drive_forward(int distance, int max_speed = 180) {

	// Get initial position 
	const int target = tracking_forward.get_position() + distance/track_wheel_size;
	// the difference between the desired position and the current distance
	double error =  target - tracking_forward.get_position();
	double prev_error;
	// accumulate total error to correct for it
	double total_error = 0;
	// The precision in mm
	const double threshold = 8;
	// The constants tuned for PID
	const double kp = .002, kd = .002, ki = .001;
	// PID control loop
	while (fabs(error)*track_wheel_size > threshold || fabs(prev_error)*track_wheel_size > threshold) {
		prev_error = error;

    if (fabs(error) < 4000) total_error += error;
    pros::lcd::set_text(7, std::to_string(total_error));

		error = target - tracking_forward.get_position();
		
		pros::lcd::set_text(1, "prev error: " + std::to_string((int)prev_error) + "       " + std::to_string(prev_error*track_wheel_size*mm_to_inch));
		pros::lcd::set_text(2, "error:      " + std::to_string((int)error) + "       " + std::to_string(error*track_wheel_size*mm_to_inch));
		pros::lcd::set_text(3, "speed: " + std::to_string(left_target));
		pros::lcd::set_text(4, "threshold: " + std::to_string(threshold/track_wheel_size));

		pros::lcd::set_text(5, "speed: "+  std::to_string((int)(kp*error)) + " + " + std::to_string((int)(kd * (error - prev_error))) + " = " + std::to_string(kp * error + kd * (error - prev_error)));
    pros::lcd::set_text(6, "ki: " + std::to_string(ki*total_error));

		left_target = kp * error + kd * (error - prev_error) + ki * total_error;
		left_target = abs(left_target) > max_speed ? max_speed*(left_target > 0 ? 1:-1) : left_target;
    chassisL(left_target);
		right_target = left_target;
    chassisR(right_target);
		pros::delay(10);
	}
	// Zero out motors so the robot does not continue moving
	left_target = 0;
	right_target = 0;
  chassisL(left_target);
  chassisR(right_target);
}

/**
 * Turns to face specific orientation
 * Function is Blocking
 *
 * @param angle the angle that the robot should turn to
 */
void turn_to(double angle) {
	double error = 180 - fmod((angle+180-(fmod(theta, 360))),360);
	double prev_error;
	double total_error = 0;
	// The precision in degrees required to exit the loop
	const double threshold = 1;
	// The constants tuned for PID
	// const double kp = 0.84, ki = 0.04, kd = 0.080;
	const double kp = 3.5, ki = 0.1, kd = 20;

	// PID control loop
	while (fabs(error) > threshold || fabs(prev_error) > threshold) {
		prev_error = error;
		error = 180 - fmod((angle + 180 - (fmod(theta, 360))), 360);
		total_error += (fabs(error) < 4.5 ? error : 0);

		pros::lcd::set_text(2, "error: " + std::to_string(error));

		left_target = -kp * error - kd * (error - prev_error) - ki * total_error;
		right_target = kp * error + kd * (error - prev_error) + ki * total_error;
    chassisL(left_target);
    chassisR(right_target);
		pros::delay(10);
	}
	// Zero out motors so the robot does not continue moving
	left_target = 0;
	right_target = 0;
  chassisL(left_target);
  chassisR(right_target);
  pros::lcd::set_text(7, "exit");

}

color get_color(double hue) {
    // Returns red or blue based off of the hue value seen by the vision sensor
    return (hue > 300 || hue < 20) ? red : blue;
}

void runRoller(int speed = 75){
    // Scores roller if the roller is at competition start position
    // Turns on flashlight
    vision.set_led_pwm(100);
    pros::lcd::set_text(4, std::to_string(vision.get_hue()));

    // Slowly backs into roller
    chassisL(-20);
    chassisR(-20);

    int counter = 0;
    bool startColor;
    startColor = get_color(vision.get_hue());

    // Spins roller until it sees a different color than what it started at    

    while (startColor == get_color(vision.get_hue()) && counter < 200) {
        roller = speed;
        counter++;
        pros::delay(10);
    }

    // Stops the intake, robot movement, and turns off flashlight
    roller = 0;
    chassisR(0);
    chassisL(0);
    vision.set_led_pwm(0);
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
  //roller = 127;
  //pros::delay(5000);
  //shoot();
  //turn_to(90);
  //drive_forward(610);
  //pros::delay(500);
  //turn_to(90);

  
  drive_forward(390);  //range between 350-400
  intakeSetting(on);
  pros::delay(700);

  drive_forward(-110);
  turn_to(197);
  pros::delay(1000);
  shoot();
  pros::delay(500);
  // turn_to(225);
  // drive_forward(-75);
  // pros::delay(200);
  turn_to(132);
  pros::delay(1000);
  drive_forward(915, 40);
  pros::delay(200);
  turn_to(222);
  drive_forward(-50);
  pros::delay(1000);
  shoot();
  //shoot the middle 3
  pros::delay(200);
  turn_to(0);
  pros::delay(200);
  drive_forward(720);
  turn_to(90);
  // 

  // //NEED TO TEST
  // 
  //
  // 
  // pros::delay(500);
  // drive_forward(-720);
  // turn_to(225);
  // pros::delay(200);
  // shoot();
  // //bar 3 (NEED to FINISH)
  // turn_to(90);
  // drive_forward(720,50);

  // //
  // pros::delay(200);
  // turn_to(197);
  // drive_forward(-915);
  // turn_to(180);
  // //Test roller
  // runRoller();





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
  // int i = 0;
  // int j =0;
  // for(i = 65; i<=10000;){
  // 	sendDataToRaspberryPi(j);
  // 	j++;
  //   controller.rumble("----...----");
  // 	pros::delay(20);
  // }

  
  pros::Task drive(driveTask);
  while (true) {

  
    // Intake
    if (controller.get_digital(DIGITAL_L1)) {
      intake_1.move(-110);
      intake_2.move(-110);
      roller = 127;
    } else if (controller.get_digital(DIGITAL_L2)) {
      intake_1.move(127);
      intake_2.move(127);
      roller = -127;
    } else {
      intake_1.brake();
      intake_2.brake();
      roller = 0;
    }


    if (controller.get_digital_new_press(DIGITAL_R1)) {
      cataFlag = true;
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

    if (controller.get_digital(DIGITAL_X)) {
      rangeSwitchToggle(true);
    }

    pros::delay(20);
  }
}
