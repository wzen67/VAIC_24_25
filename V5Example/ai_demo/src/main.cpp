/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "ai_functions.h"

using namespace vex;

brain Brain;
// Robot configuration code.
motor leftDrive = motor(PORT1, ratio18_1, false);
motor rightDrive = motor(PORT2, ratio18_1, true);
gps GPS = gps(PORT12, -127, -165, distanceUnits::mm, 180);
smartdrive Drivetrain = smartdrive(leftDrive, rightDrive, GPS, 319.19, 320, 40, mm, 1);
// Controls arm used for raising and lowering rings
motor Arm = motor(PORT3, ratio18_1, false);
// Controls the chain at the front of the arm
// used for pushing rings off of the arm
motor Chain = motor(PORT8, ratio18_1, false);


// A global instance of competition
competition Competition;

// create instance of jetson class to receive location and other
// data from the Jetson nano
//
ai::jetson  jetson_comms;

/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
// #define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT10, "robot_32456_1", linkType::manager );
#else
#pragma message("building for the worker")
ai::robot_link       link( PORT10, "robot_32456_1", linkType::worker );
#endif

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auto_Isolation(void) {
  // Calibrate GPS Sensor
  GPS.calibrate();
  // Optional wait to allow for calibration
  waitUntil(!(GPS.isCalibrating()));

  // Set brake mode for the arm
  Arm.setStopping(brakeType::hold);
  // Reset the position of the arm while its still on the ground
  Arm.resetPosition();
  // Lift the arm to prevent dragging
  Arm.spinTo(75, rotationUnits::deg);

  // Finds and moves robot to over the closest blue ring
  goToObject(OBJECT::BlueRing);
  grabRing();
  // Find and moves robot to the closest mobile drop
  // then drops the ring on the goal
  goToObject(OBJECT::MobileGoal);
  dropRing();
  // Back off from the goal
  Drivetrain.driveFor(-30, distanceUnits::cm);

}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                        Auto_Interaction Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous interaction*/
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void auto_Interaction(void) {
  // Add functions for interaction phase
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = true;

void autonomousMain(void) {
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period. 
  // ..........................................................................

  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();

  firstAutoFlag = false;
}


int main() {
  // local storage for latest data from the Jetson Nano
  static AI_RECORD local_map;

  // Run at about 15Hz
  int32_t loop_time = 33;

  // Setup debug screen
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Starting main()...");

  // Start the status update display (Jetson + VEXlink dashboard)
  Brain.Screen.newLine(); Brain.Screen.print("Starting dashboard...");
  thread t1(dashboardTask);

  // Setup autonomous callback
  Brain.Screen.newLine(); Brain.Screen.print("Setting up Competition mode...");
  Competition.autonomous(autonomousMain);

  this_thread::sleep_for(loop_time);
  Arm.setVelocity(60, percent);
  Brain.Screen.newLine(); Brain.Screen.print("Entering main loop...");

  while(1) {
    // Get last map data from Jetson
    jetson_comms.get_data(&local_map);

    // Print detection count
    Brain.Screen.setCursor(7, 1);  // Fixed position to overwrite same line
    Brain.Screen.print("Detections: %d     ", local_map.detectionCount);

    // Print first detected object info if available
    if (local_map.detectionCount > 0) {
      DETECTION_OBJECT &obj = local_map.detections[0];
      Brain.Screen.setCursor(8, 1);
      Brain.Screen.print("ClassID: %d Prob: %.2f", obj.classID, obj.probability);
    }

    // Set location to send to partner robot
    link.set_remote_location(local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status);

    // Request new map data
    jetson_comms.request_map();

    // Wait for next loop
    this_thread::sleep_for(loop_time);
  }

  
  // TEMPORARY: force autonomous run //debug only
  // auto_Isolation(); 
}