/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       kyleliu                                                   */
/*    Created:      11/22/2025, 8:51:49 PM                                    */
/*    Description:  IQ2 project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the IQ2 brain screen
vex::brain Brain;

// VEXcode device constructors
controller Controller = controller();
motor LeftMotor = motor(PORT8, false);
motor RightMotor = motor(PORT2, true);
motor motor2 = motor(PORT2, true);
motor motor7 = motor(PORT7, true);
motor motor1 = motor(PORT1, false);
motor motor8 = motor(PORT8, false);
motor motor6 = motor(PORT6, false);
motor motor12 = motor(PORT12, true);
distance leftclawdistancesensor = distance(PORT4);
distance rightclawdistancesensor = distance(PORT10);
distance distancesensorightclaw = distance(PORT11);
distance distancesensorleftclaw = distance(PORT5);
pneumatic P1 = pneumatic(PORT4);
// from front view left
pneumatic P2 = pneumatic(PORT10);
distance distancesensorfourbar = distance(PORT6);
float Rmotorspeed, Lmotorspeed, Rightstick, Leftstick, deadband = 10;
bool crawlmodestate;
bool isfrontclawrightopen;
bool isfrontclawleftopen;
bool isfingeropen;
bool isbackarmup;
bool isclawsensorsdetecting;
int distancedetection = 80;
bool isstandoffgoalstacking;
bool isfrontclawup;
bool startingpingrabstate;
int crawlspeed = 35;
void lowerfromstandoffgoal();
event eventfrontclawgodown = event();
thread drivetrainthread = thread();
event eventraisebackarmtonest = event();
// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

void pumpon() {
  P1.pumpOn();
  P2.pumpOn();
}

void setupstartingpingrabbing() { P1.extend(cylinder2); }

void retractstartingpingrabbing() { P1.retract(cylinder2); }

void frontclawleftclose() {
  P2.retract(cylinder2);
  isfrontclawleftopen = false;
}

void frontclawleftopen() {
  P2.extend(cylinder2);
  isfrontclawleftopen = true;
}

void frontclawrightclose() {
  P1.retract(cylinder1);
  isfrontclawrightopen = false;
}
void frontclawrightopen() {
  P1.extend(cylinder1);
  isfrontclawrightopen = true;
}

void frontclawopen() {
  frontclawleftopen();
  frontclawrightopen();
}

void closefinger() {
  P2.extend(cylinder1);
  isfingeropen = false;
}

void openfinger() {
  P2.retract(cylinder1);
  isfingeropen = true;
}

void spinbackarmup() {
  motor1.setStopping(hold);
  motor7.setStopping(hold);
  motor1.spin(forward);
  motor7.spin(forward);
}

void spinbackarmdown() {
  motor1.setStopping(coast);
  motor7.setStopping(coast);
  motor1.spin(reverse);
  motor7.spin(reverse);
}

void backarmstop() {
  motor1.stop();
  motor7.stop();
}

void movefrontclawup() {
  isfrontclawup = true;
  motor6.spin(forward);
  motor12.spin(forward);
}

void movefrontclawdown() {
  isfrontclawup = false;
  motor6.spin(reverse);
  motor12.spin(reverse);
}

void frontclawstop() {
  motor6.stop();
  motor12.stop();
}

void fingercontrol() {
  if (isfingeropen) {
    // spinbackarmdown();
    // wait(100, msec);
    closefinger();
    wait(0.5, seconds);
    eventraisebackarmtonest.broadcast();
    // wait(0.2, seconds);
    // spinbackarmup();
    // while (motor1.position(degrees) < 139) {
    //   wait(20, msec);
    // }
    // motor1.setStopping(hold);
    // motor7.setStopping(hold);
    // backarmstop();
  } else {
    motor1.setStopping(coast);
    motor7.setStopping(coast);
    openfinger();
    spinbackarmdown();
    while (motor1.position(degrees) > 5) {
      wait(20, msec);
    }

    backarmstop();
  }
}

void dumppinsontobeam() {
  closefinger();
  motor6.setStopping(hold);
  motor12.setStopping(hold);
  movefrontclawup();
  while (motor6.position(degrees) < 520) { // 625
    wait(20, msec);
  }
  motor6.setStopping(coast);
  motor12.setStopping(coast);
  motor6.setVelocity(0, percent);
  motor12.setVelocity(0, percent);
  while (motor6.position(degrees) < 720) {
    wait(20, msec);
  }
  frontclawstop();
  isclawsensorsdetecting = false;
  frontclawopen();
  motor12.setVelocity(100, percent);
  motor6.setVelocity(100, percent);
  wait(0.2, seconds);
  eventfrontclawgodown.broadcast();
  wait(0.2, seconds);
  // spinbackarmdown();
  // wait(0.2, seconds);
  motor1.setStopping(hold);
  motor7.setStopping(hold);
  backarmstop();
  wait(1, seconds);
  motor6.setStopping(coast);
  motor12.setStopping(coast);
  frontclawstop();
  // eventraisebackarmtonest.broadcast();
  isclawsensorsdetecting = true;
}

// ==============================================================================
// ARM FUNCTIONS
// ==============================================================================

void raisebackarmtostandoff() {
  isbackarmup = true;
  closefinger();
  spinbackarmup();
  while (motor1.position(degrees) < 1240) {
    wait(20, msec);
  }
  backarmstop();
}

void lowerbackarmfromstandoff() {
  isbackarmup = false;
  spinbackarmdown();
  while (motor1.position(degrees) > 1080) {
    wait(20, msec);
  }
  openfinger();
  while (motor1.position(degrees) > 10) {
    wait(20, msec);
  }
  backarmstop();
}

void backarmcontrol() {
  if (isbackarmup) {
    lowerbackarmfromstandoff();
  } else {
    raisebackarmtostandoff();
  }
}

// ==============================================================================
// DRIVER FUNCTIONS
// ==============================================================================

void splitdrivewithcrawlmode() {
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
  while (true) {
    LeftMotor.setStopping(brake);
    RightMotor.setStopping(brake);
    int absA = fabs(Controller.AxisA.position());
    int absC = fabs(Controller.AxisC.position());
    float A_position = Controller.AxisA.position();
    float C_position = Controller.AxisC.position();
    if (crawlmodestate) {
      LeftMotor.setStopping(hold);
      RightMotor.setStopping(hold);
      if (A_position > deadband) {
        A_position = crawlspeed;
      } else if (A_position < deadband * (-1)) {
        A_position = crawlspeed * (-1);
      } else {
        A_position = 0;
      }
    }

    if (crawlmodestate) {
      if (C_position > deadband) {
        C_position = crawlspeed;
      } else if (C_position < deadband * (-1)) {
        C_position = crawlspeed * (-1);
      } else {
        C_position = 0;
      }
    }
    float LeftSpeed = A_position + C_position;
    float RightSpeed = A_position - C_position;

    if ((absA + absC) > deadband) {
      LeftMotor.setVelocity(LeftSpeed, percent);
      RightMotor.setVelocity(RightSpeed, percent);
    } else {
      LeftMotor.setVelocity(0, percent);
      RightMotor.setVelocity(0, percent);
    }
    LeftMotor.spin(forward);
    RightMotor.spin(forward);
    wait(20, msec);
  }
}

// ==============================================================================
// MAIN PROGRAM
// ==============================================================================

void inital() {
  isbackarmup = false;
  isclawsensorsdetecting = true;
  isfrontclawleftopen = true;
  isfrontclawrightopen = true;
  isfingeropen = true;
  openfinger();
  frontclawopen();
  motor1.setStopping(hold);
  motor7.setStopping(hold);
  LeftMotor.setMaxTorque(100, percent);
  RightMotor.setMaxTorque(100, percent);
  motor2.setStopping(hold);
  motor8.setStopping(hold);
  motor1.setVelocity(100, percent);
  motor7.setVelocity(100, percent);
  motor1.setMaxTorque(100, percent);
  motor7.setMaxTorque(100, percent);
  motor2.setVelocity(100, percent);
  motor8.setVelocity(100, percent);
  motor2.setMaxTorque(100, percent);
  motor8.setMaxTorque(100, percent);
  motor6.setVelocity(100, percent);
  motor12.setVelocity(100, percent);
  motor12.setMaxTorque(100, percent);
  motor6.setMaxTorque(100, percent);
  spinbackarmdown();
  wait(0.2, seconds);
  backarmstop();
  motor1.setPosition(0, degrees);
  motor7.setPosition(0, degrees);
}

void stackpins() {
  motor6.setStopping(hold);
  motor12.setStopping(hold);
  if (isfrontclawup) {
    movefrontclawdown();
    while (motor6.position(degrees) > 5) {
      wait(20, msec);
    }
    frontclawopen();
    frontclawstop();
    wait(0.2, seconds);
    isclawsensorsdetecting = true;
  } else {
    isclawsensorsdetecting = false;
    movefrontclawup();
    while (motor6.position(degrees) < 175) {
      wait(20, msec);
    }
    frontclawstop();
  }
}

void raisebackarmtonest() {
  spinbackarmup();
  motor1.setVelocity(50, percent);
  motor7.setVelocity(50, percent);
  while (motor1.position(degrees) < 127) {
    wait(20, msec);
  }
  backarmstop();
  motor1.setVelocity(100, percent);
  motor7.setVelocity(100, percent);
}

void grabstartingpin() {
  isclawsensorsdetecting = false;
  startingpingrabstate = !startingpingrabstate;
  if (startingpingrabstate) {
    frontclawopen();
    movefrontclawup();
    while (motor6.position(degrees) < 100) {
      wait(20, msec);
    }
    frontclawstop();
  } else {
    movefrontclawdown();
    while (motor6.position(degrees) > 5) {
      wait(20, msec);
    }
    frontclawstop();
  }
  wait(0.5, seconds);
  isclawsensorsdetecting = true;
}

void stackpinsontostandoff() {
  isstandoffgoalstacking = !isstandoffgoalstacking;
  isclawsensorsdetecting = false;
  if (isstandoffgoalstacking) {
    movefrontclawup();
    while (motor6.position(degrees) < 280) {
      wait(20, msec);
    }
    motor12.setStopping(hold);
    motor6.setStopping(hold);
    frontclawstop();
  } else {
    movefrontclawdown();
    while (motor6.position(degrees) > 190) {
      wait(20, msec);
    }
    frontclawopen();
    while (motor6.position(degrees) > 5) {
      wait(20, msec);
    }
    motor12.setStopping(coast);
    motor6.setStopping(coast);
    frontclawstop();
  }
  wait(0.5, seconds);
  isclawsensorsdetecting = true;
}

int main() {
  inital();
  pumpon();
  drivetrainthread = thread(splitdrivewithcrawlmode);
  eventraisebackarmtonest = event(raisebackarmtonest);
  Controller.ButtonLUp.pressed(frontclawopen);
  Controller.ButtonRDown.pressed(fingercontrol);
  Controller.ButtonFUp.pressed(backarmcontrol);
  Controller.ButtonEUp.pressed(dumppinsontobeam);
  Controller.ButtonEDown.pressed(stackpinsontostandoff);
  Controller.ButtonLDown.pressed(stackpins);
  Controller.ButtonFDown.pressed(grabstartingpin);
  Brain.playSound(tada);
  eventfrontclawgodown = event(movefrontclawdown);
  // Run main drive control loop
  while (true) {
    // printf("positioning of A is %d\n", Controller.AxisA.position());
    // printf("positioning of B is %d\n", Controller.AxisB.position());
    // printf("\033[2J\n");
    // printf("\n");
    if (Controller.ButtonRUp.pressing()) {
      crawlmodestate = true;
      Brain.playSound(siren);
    } else {
      crawlmodestate = false;
    }

    if (!Controller.ButtonLUp.pressing()) {
      if (isclawsensorsdetecting) {
        if (distancesensorleftclaw.objectDistance(mm) < distancedetection) {
          if (isfrontclawleftopen) {
            Brain.playSound(doorClose);
            frontclawleftclose();
          }
        }
        if (distancesensorightclaw.objectDistance(mm) < distancedetection) {
          if (isfrontclawrightopen) {
            Brain.playSound(doorClose);
            frontclawrightclose();
          }
        }
      }
    }

    // printf("left distance sensor is %.2f\n", isrightclawfilled());
    wait(20, msec);
  }
}
