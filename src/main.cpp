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
motor LeftMotor = motor(PORT9, false);
motor RightMotor = motor(PORT3, true);
// motor motor2 = motor(PORT2, true);
touchled touchled5 = touchled(PORT5);
motor BackArmMotor1 = motor(PORT2, true);
motor BackArmMotor2 = motor(PORT8, false);
motor_group BackArmMotorGroup = motor_group(BackArmMotor1, BackArmMotor2);
motor motor2 = motor(PORT2, false);
motor motor8 = motor(PORT8, true);
motor FrontArmMotor1 = motor(PORT4, true);
motor FrontArmMotor2 = motor(PORT10, false);
motor_group frontarmmotorgroup = motor_group(FrontArmMotor1, FrontArmMotor2);
pneumatic P1 = pneumatic(PORT11);
// from front view left
pneumatic P2 = pneumatic(PORT12);
pneumatic P3 = pneumatic(PORT6);
float Rmotorspeed, Lmotorspeed, Rightstick, Leftstick, deadband = 10;
bool crawlmodestate = false;
float Dinput;
bool isfrontclawdownstate = false;
bool Stackingbuttonstate;
bool driveforwardstate = false;
bool iscornergoalstacking;
bool isfrontclawrightopen;
bool isfrontclawleftopen;
bool isfingeropen;
bool touchledstate;
int standoffheightstatecounter;
bool isbackarmup;
bool ispusherextended;
int frontclawstandoffheight;
int cornergoalstackingheight = 100;
int distancedetection = 80;
float backarmloweringheight = 475;
int frontclawcounterstate = 0;
float now;
float duration;
float start;
void stack91onstandoff();
bool guidestate;
int backarmstatecounter;
bool isfrontclawup;
bool isbackarmloweringusingbutton;
bool startingpingrabstate;
bool isclawgrabbed = true;
bool clawstartingcontrol;
int crawlspeed = 35;
void lowerfromstandoffgoal();
void splitdrivewithcrawlmode();
void stack91();
void stack110();
void stack121();
void lowerfrontarmfromstandoffheight();
event eventbackclawgodownforbeam = event();
thread drivetrainthread = thread();
event eventraisebackarmtogroundystack = event();
event eventraisebackarmtoministandoffstack = event();
event eventraisebackarmtobigstandoffystack = event();
event eventraisebackarmtonest = event();
event eventfrontclawgodown = event();
event eventdriveforwardforstandoff = event();
thread fingergrabgoingupthread = thread();
thread raisebackarmthread = thread();
thread threadfrontclawcontrol = thread();
// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

void killdrivetrain() {
  drivetrainthread.interrupt();
  // LeftMotor.setStopping(hold);
  // RightMotor.setStopping(hold);
  LeftMotor.stop();
  RightMotor.stop();
  LeftMotor.setVelocity(0, percent);
  RightMotor.setVelocity(0, percent);
}

void deployguide() {
  P1.extend(cylinder1);
  // P3.extend(cylinder1);
}

void retractguide() {
  P1.retract(cylinder1);
  // P3.retract(cylinder1);
}

void pumpon() {
  P1.pumpOn();
  P2.pumpOn();
  // P3.pumpOn();
}
void extendclawbalancer() { P2.extend(cylinder1); }
void retractclawbalancer() { P2.retract(cylinder1); }

void extendpusher() { P1.extend(cylinder2); }
void retractpusher() { P1.retract(cylinder2); }

void frontclawleftclose() {
  P1.retract(cylinder2);
  isfrontclawleftopen = false;
}

void frontclawleftopen() {
  P1.extend(cylinder2);
  isfrontclawleftopen = true;
}

void frontclawopen() { frontclawleftopen(); }

void closefinger() {
  P2.extend(cylinder2);
  isfingeropen = false;
}

void openfinger() {
  P2.retract(cylinder2);
  isfingeropen = true;
}

void spinbackarmup() {
  BackArmMotorGroup.setStopping(hold);
  BackArmMotorGroup.spin(forward);
}

void spinbackarmdown() { BackArmMotorGroup.spin(reverse); }

void backarmstop() { BackArmMotorGroup.stop(); }

void movefrontclawup() { frontarmmotorgroup.spin(forward); }

void movefrontclawdown() { frontarmmotorgroup.spin(reverse); }

void frontclawstop() { frontarmmotorgroup.stop(); }

void fingercontrol() {
  if (isfingeropen) {
    LeftMotor.stop();
    RightMotor.stop();
    spinbackarmdown();
    wait(0.1, seconds);
    closefinger();
    wait(0.5, seconds);
    spinbackarmup();
    while (BackArmMotorGroup.position(degrees) < 80) {
      wait(20, msec);
    }
    BackArmMotorGroup.setStopping(hold);
    backarmstop();
  } else {
    BackArmMotorGroup.stop();
    if (backarmstatecounter % 3 == 1) {
      drivetrainthread.interrupt();
      LeftMotor.stop();
      RightMotor.stop();
      stack91();
    } else if (backarmstatecounter % 3 == 2) {
      if (standoffheightstatecounter % 3 == 1) {
        while (Controller.ButtonRDown.pressing()) {
          Brain.playSound(siren);
          if (Controller.AxisD.position() > 80) {
            BackArmMotorGroup.setVelocity(60, percent);
            BackArmMotorGroup.spin(forward);
            isbackarmloweringusingbutton = true;
            wait(0.2, seconds);
          } else if (Controller.AxisD.position() < -80) {
            BackArmMotorGroup.setVelocity(60, percent);
            BackArmMotorGroup.spin(reverse);
          } else {
            BackArmMotorGroup.setStopping(hold);
            BackArmMotorGroup.stop();
            BackArmMotorGroup.setVelocity(100, percent);
          }
          wait(20, msec);
          BackArmMotorGroup.stop();
        }
        isbackarmloweringusingbutton = false;
        stack121();
      } else if (standoffheightstatecounter % 3 == 2) {
        touchled5.setColor(yellow_green);
        stack110();
        backarmstatecounter = 0;
      } else {
        stack84();
        backarmstatecounter = 0;
      }
    } else {
      BackArmMotorGroup.setStopping(coast);
      BackArmMotorGroup.setVelocity(30, percent);
      spinbackarmdown();
      LeftMotor.setVelocity(0, percent);
      RightMotor.setVelocity(0, percent);
      wait(0.54, seconds);
      BackArmMotorGroup.stop();
      BackArmMotorGroup.setVelocity(100, percent);
      wait(0.2, seconds);
      openfinger();
      wait(0.15, seconds);
      spinbackarmup();
      if (BackArmMotorGroup.position(degrees) > 100) {
        while (BackArmMotorGroup.position(degrees) < 170) {
          wait(20, msec);
        }
      }
      backarmstop();
      // if (BackArmMotorGroup.position(degrees) > 100) {
      //   drivetrainthread.interrupt();
      //   LeftMotor.setVelocity(100, percent);
      //   RightMotor.setVelocity(100, percent);
      //   RightMotor.spin(forward);
      //   LeftMotor.spin(forward);
      //   wait(0.45, seconds);
      //   drivetrainthread = thread(splitdrivewithcrawlmode);
      // }
      // wait(0.25, seconds);
      spinbackarmdown();
      while (BackArmMotorGroup.position(degrees) > 20) {
        wait(20, msec);
      }
      BackArmMotorGroup.setStopping(coast);
      BackArmMotorGroup.setVelocity(100, percent);
      backarmstop();
      backarmstatecounter = 0;
    }
  }
}

void backarmgodownforbeam() {
  spinbackarmdown();
  wait(0.45, seconds);
  if (!Controller.ButtonFUp.pressing()) {
    spinbackarmup();
    while (BackArmMotorGroup.position(degrees) < 80) {
      wait(20, msec);
    }
    BackArmMotorGroup.setStopping(hold);
    backarmstop();
  }
}

void dumppinsontobeam() {
  retractclawbalancer();
  frontclawleftclose();
  wait(0.1, seconds);
  closefinger();
  eventbackclawgodownforbeam.broadcast();
  movefrontclawup();
  frontarmmotorgroup.setVelocity(100, percent);
  while (frontarmmotorgroup.position(degrees) < 480) { // 625
    wait(20, msec);
  }
  frontarmmotorgroup.setStopping(brake);
  frontarmmotorgroup.setVelocity(100, percent);
  retractclawbalancer();
  while (frontarmmotorgroup.position(degrees) < 700) { // 740
    wait(20, msec);
  }
  frontclawopen();
  isclawgrabbed = true;
  frontclawstop();
  frontarmmotorgroup.setVelocity(100, percent);
  eventfrontclawgodown.broadcast();
  while (frontarmmotorgroup.position(degrees) > 5) {
    wait(20, msec);
  }
  extendclawbalancer();
  frontarmmotorgroup.setStopping(brake);
  frontclawleftopen();
  iscornergoalstacking = false;
  isfrontclawup = true;
  isfrontclawdownstate = true;
  wait(0.1, seconds);
  frontclawstop();
}

// ==============================================================================
// ARM FUNCTIONS
// ==============================================================================

void lowerbackarmfromstandoff() {
  spinbackarmdown();
  while (BackArmMotorGroup.position(degrees) > 1080) {
    wait(20, msec);
  }
  openfinger();
  while (BackArmMotorGroup.position(degrees) > 10) {
    wait(20, msec);
  }
  backarmstop();
}
void lowerbackarmfromstandoffwithoutrelease() {
  BackArmMotorGroup.setVelocity(60, percent);
  spinbackarmdown();
  // while (BackArmMotorGroup.position(degrees) > 10) {
  //   wait(20, msec);
  // }
  // backarmstop();
  // BackArmMotorGroup.setVelocity(100, percent);
  // spinbackarmup();
  while (BackArmMotorGroup.position(degrees) > 100) {
    wait(20, msec);
  }
  backarmstop();
}
void raisebackarmtogroundystack() {
  if (BackArmMotorGroup.position(degrees) > 300) {
    spinbackarmdown();
    while (BackArmMotorGroup.position(degrees) > 315) {
      wait(20, msec);
    }
    BackArmMotorGroup.setStopping(hold);
    backarmstop();
    deployguide();
  } else {
    spinbackarmup();
    while (BackArmMotorGroup.position(degrees) < 315) {
      wait(20, msec);
    }
    backarmstop();
    deployguide();
  }
}

void raisebackarmtoministandoffstack() {
  if (BackArmMotorGroup.position(degrees) > 400) {
    spinbackarmdown();
    while (BackArmMotorGroup.position(degrees) > 440) {
      wait(20, msec);
    }
    BackArmMotorGroup.setStopping(hold);
    backarmstop();
  } else {
    spinbackarmup();
    while (BackArmMotorGroup.position(degrees) < 440) {
      wait(20, msec);
    }
    backarmstop();
  }
}

void raisebackarmtobigstandoffystack() {
  if (BackArmMotorGroup.position(degrees) > 450) {
    spinbackarmdown();
    while (BackArmMotorGroup.position(degrees) < 455) {
      wait(20, msec);
    }
    BackArmMotorGroup.setStopping(hold);
    // wait(0.25, seconds);
    backarmstop();
  } else {
    spinbackarmup();

    while (BackArmMotorGroup.position(degrees) < 495) { // 455
      wait(20, msec);
    }
    wait(0.175, seconds);
    backarmstop();
  }
}

// void raisebackarmtobigstandoffystack() {
//   if (BackArmMotor1.position(degrees) > 450) {
//     spinbackarmdown();
//     while (BackArmMotor1.position(degrees) < 455) {
//       wait(20, msec);
//     }
//     BackArmMotor1.setStopping(hold);
//     BackArmMotor2.setStopping(hold);
//     wait(0.25, seconds);
//     backarmstop();
//   } else {
//     spinbackarmup();
//     while (BackArmMotor1.position(degrees) < 515) { // 455
//       wait(20, msec);
//     }
//     wait(0.22, seconds);
//     backarmstop();
//   }
// }

void stack84() {
  drivetrainthread.interrupt();
  LeftMotor.setVelocity(30, percent);
  RightMotor.setVelocity(30, percent);
  LeftMotor.spin(forward);
  RightMotor.spin(forward);
  wait(0.41, seconds);
  drivetrainthread = thread(splitdrivewithcrawlmode);
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(60, percent);
  spinbackarmdown();
  wait(0.58, seconds);
  BackArmMotorGroup.stop();
  BackArmMotorGroup.setVelocity(100, percent);
  while (Controller.ButtonRDown.pressing()) {
    Brain.playSound(siren);
    if (Controller.AxisD.position() > 30) {
      BackArmMotorGroup.setVelocity(60, percent);
      BackArmMotorGroup.spin(forward);
      isbackarmloweringusingbutton = true;
      wait(0.2, seconds);
    } else if (Controller.AxisD.position() < -30) {
      BackArmMotorGroup.setVelocity(60, percent);
      BackArmMotorGroup.spin(reverse);
    } else {
      BackArmMotorGroup.setStopping(hold);
      BackArmMotorGroup.stop();
      BackArmMotorGroup.setVelocity(100, percent);
    }
    wait(20, msec);
    BackArmMotorGroup.stop();
  }
  openfinger();
  spinbackarmup();
  if (BackArmMotorGroup.position(degrees) > 380) {
    while (BackArmMotorGroup.position(degrees) < 440) {
      wait(20, msec);
    }
  }
  wait(0.2, seconds);
  backarmstop();
  if (BackArmMotorGroup.position(degrees) > 100) {
    drivetrainthread.interrupt();
    LeftMotor.setVelocity(100, percent);
    RightMotor.setVelocity(100, percent);
    RightMotor.spin(forward);
    LeftMotor.spin(forward);
    wait(0.4, seconds);
    drivetrainthread = thread(splitdrivewithcrawlmode);
  }
  wait(0.25, seconds);
  spinbackarmdown();
  while (BackArmMotorGroup.position(degrees) > 20) {
    wait(20, msec);
  }
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(100, percent);
  backarmstop();
  backarmstatecounter = 0;
}

void stack121() {
  drivetrainthread.interrupt();
  LeftMotor.stop();
  RightMotor.stop();
  BackArmMotorGroup.setVelocity(30, percent);
  spinbackarmdown();
  while (BackArmMotorGroup.position(degrees) > 480) {
    wait(20, msec);
  }
  BackArmMotorGroup.stop();
  BackArmMotorGroup.setVelocity(100, percent);
  openfinger();
  wait(0.1, seconds);
  LeftMotor.setVelocity(100, percent);
  RightMotor.setVelocity(100, percent);
  RightMotor.spin(forward);
  LeftMotor.spin(forward);
  wait(0.5, seconds);
  drivetrainthread = thread(splitdrivewithcrawlmode);
  spinbackarmdown();
  while (BackArmMotorGroup.position(degrees) > 100) {
    wait(20, msec);
  }
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(100, percent);
  backarmstop();
  backarmstatecounter = 0;
  isclawgrabbed = true;
}

void stack110() {
  drivetrainthread.interrupt();
  LeftMotor.setVelocity(30, percent);
  RightMotor.setVelocity(30, percent);
  LeftMotor.spin(forward);
  RightMotor.spin(forward);
  wait(0.41, seconds);
  drivetrainthread = thread(splitdrivewithcrawlmode);
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(60, percent);
  spinbackarmdown();
  wait(0.58, seconds);
  BackArmMotorGroup.stop();
  BackArmMotorGroup.setVelocity(100, percent);
  while (Controller.ButtonRDown.pressing()) {
    Brain.playSound(siren);
    if (Controller.AxisD.position() > 30) {
      BackArmMotorGroup.setVelocity(60, percent);
      BackArmMotorGroup.spin(forward);
      isbackarmloweringusingbutton = true;
      wait(0.2, seconds);
    } else if (Controller.AxisD.position() < -30) {
      BackArmMotorGroup.setVelocity(60, percent);
      BackArmMotorGroup.spin(reverse);
    } else {
      BackArmMotorGroup.setStopping(hold);
      BackArmMotorGroup.stop();
      BackArmMotorGroup.setVelocity(100, percent);
    }
    wait(20, msec);
    BackArmMotorGroup.stop();
  }
  openfinger();
  spinbackarmup();
  if (BackArmMotorGroup.position(degrees) > 380) {
    while (BackArmMotorGroup.position(degrees) < 440) {
      wait(20, msec);
    }
  }
  wait(0.2, seconds);
  backarmstop();
  if (BackArmMotorGroup.position(degrees) > 100) {
    drivetrainthread.interrupt();
    LeftMotor.setVelocity(100, percent);
    RightMotor.setVelocity(100, percent);
    RightMotor.spin(forward);
    LeftMotor.spin(forward);
    wait(0.4, seconds);
    drivetrainthread = thread(splitdrivewithcrawlmode);
  }
  wait(0.25, seconds);
  spinbackarmdown();
  while (BackArmMotorGroup.position(degrees) > 20) {
    wait(20, msec);
  }
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(100, percent);
  backarmstop();
  backarmstatecounter = 0;
}

void stack91() {
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(30, percent);
  spinbackarmdown();
  wait(0.45, seconds);
  BackArmMotorGroup.stop();
  // wait(0.2, seconds);
  BackArmMotorGroup.setVelocity(100, percent);
  while (Controller.ButtonRDown.pressing()) {
    drivetrainthread = thread(splitdrivewithcrawlmode);
    Brain.playSound(siren);
    if (Controller.AxisD.position() > 30) {
      BackArmMotorGroup.setVelocity(60, percent);
      BackArmMotorGroup.spin(forward);
      isbackarmloweringusingbutton = true;
      wait(0.2, seconds);
    } else if (Controller.AxisD.position() < -30) {
      if (BackArmMotorGroup.position(degrees) > 270) {
        BackArmMotorGroup.setVelocity(60, percent);
        BackArmMotorGroup.spin(reverse);
      }
    } else {
      BackArmMotorGroup.setStopping(hold);
      BackArmMotorGroup.stop();
      BackArmMotorGroup.setVelocity(100, percent);
    }
    wait(20, msec);
    BackArmMotorGroup.stop();
  }
  drivetrainthread.interrupt();
  LeftMotor.stop();
  RightMotor.stop();
  openfinger();
  // wait(0.1, seconds);
  wait(0.5, seconds);
  spinbackarmup();
  if (BackArmMotorGroup.position(degrees) > 100) {
    while (BackArmMotorGroup.position(degrees) < 170) {
      wait(20, msec);
    }
  }
  wait(0.25, seconds);
  backarmstop();
  retractguide();
  if (BackArmMotorGroup.position(degrees) > 100) {
    // spinbackarmup();
    // wait(0.4, seconds);
    // backarmstop();
    // drivetrainthread.interrupt();
    LeftMotor.setVelocity(100, percent);
    RightMotor.setVelocity(100, percent);
    RightMotor.spin(forward);
    LeftMotor.spin(forward);
    wait(0.4, seconds);
    drivetrainthread = thread(splitdrivewithcrawlmode);
  }
  wait(0.25, seconds);
  spinbackarmdown();
  while (BackArmMotorGroup.position(degrees) > 20) {
    wait(20, msec);
  }
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(100, percent);
  backarmstop();
  backarmstatecounter = 0;
}

void raisebackarmto91onstandoff() {
  if (BackArmMotorGroup.position(degrees) > 330) {
    spinbackarmdown();
    while (BackArmMotorGroup.position(degrees) > 330) {
      wait(20, msec);
    }
    BackArmMotorGroup.setStopping(hold);
    backarmstop();
  } else {
    spinbackarmup();
    while (BackArmMotorGroup.position(degrees) < 330) {
      wait(20, msec);
    }
    wait(0.15, seconds);
    backarmstop();
  }
}

// void stack91onstandoff() {
//   drivetrainthread.interrupt();
//   LeftMotor.setVelocity(30, percent);
//   RightMotor.setVelocity(30, percent);
//   LeftMotor.spin(forward);
//   RightMotor.spin(forward);
//   wait(0.45, seconds);
//   drivetrainthread = thread(splitdrivewithcrawlmode);
//   BackArmMotorGroup.setStopping(coast);
//   BackArmMotorGroup.setVelocity(30, percent);
//   spinbackarmdown();
//   wait(0.54, seconds);
//   BackArmMotorGroup.stop();
//   BackArmMotorGroup.setVelocity(100, percent);
//   wait(0.15, seconds);
//   openfinger();
//   spinbackarmup();
//   if (BackArmMotorGroup.position(degrees) > 320) {
//     while (BackArmMotorGroup.position(degrees) < 380) {
//       wait(20, msec);
//     }
//   }
//   wait(0.2, seconds);
//   backarmstop();
//   if (BackArmMotorGroup.position(degrees) > 100) {
//     // spinbackarmup();
//     // wait(0.4, seconds);
//     // backarmstop();
//     drivetrainthread.interrupt();
//     LeftMotor.setVelocity(100, percent);
//     RightMotor.setVelocity(100, percent);
//     RightMotor.spin(forward);
//     LeftMotor.spin(forward);
//     wait(0.4, seconds);
//     drivetrainthread = thread(splitdrivewithcrawlmode);
//   }
//   wait(0.25, seconds);
//   spinbackarmdown();
//   while (BackArmMotorGroup.position(degrees) > 20) {
//     wait(20, msec);
//   }
//   BackArmMotorGroup.setStopping(coast);
//   BackArmMotorGroup.setVelocity(100, percent);
//   backarmstop();
//   backarmstatecounter = 0;
// }

void backarmcontrol() {
  backarmstatecounter++;
  raisebackarmthread.interrupt();
  if (backarmstatecounter % 3 == 1) { // raise to 91 height
    fingergrabgoingupthread.interrupt();
    closefinger();
    raisebackarmthread = thread(raisebackarmtogroundystack);
    touchled5.setColor(green);
  } else if (backarmstatecounter % 3 == 2) { // raise to 121 height
    retractguide();
    raisebackarmthread = thread(raisebackarmtobigstandoffystack);
    touchled5.setColor(red);
  } else if (backarmstatecounter % 3 == 0) {
    raisebackarmthread = thread(lowerbackarmfromstandoffwithoutrelease);
    touchled5.setColor(white);
  }
}

// ==============================================================================
// DRIVER FUNCTIONS
// ==============================================================================

void splitdrivewithcrawlmode() {
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
  while (true) {

    int absA = fabs(Controller.AxisA.position());
    int absC = fabs(Controller.AxisC.position());
    float A_position = Controller.AxisA.position();
    float C_position = Controller.AxisC.position();
    if (crawlmodestate) {
      // LeftMotor.setStopping(hold);
      // RightMotor.setStopping(hold);
      if (Controller.ButtonRDown.pressing()) {
        deadband = 70;
      } else {
        deadband = 10;
      }
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
    if (!Controller.ButtonLDown.pressing()) {
      if (Controller.ButtonRDown.pressing()) {
        C_position = C_position * 0.2;
        printf("in function");
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
  isfrontclawdownstate = true;
  // P3.extend(cylinder1);
  isbackarmup = false;
  isfrontclawleftopen = true;
  isfrontclawrightopen = true;
  isfingeropen = true;
  isfrontclawup = true;
  frontclawcounterstate = 0;
  openfinger();
  frontclawopen();
  BackArmMotorGroup.setStopping(hold);
  frontarmmotorgroup.setPosition(0, degrees);
  LeftMotor.setMaxTorque(100, percent);
  RightMotor.setMaxTorque(100, percent);
  frontarmmotorgroup.setVelocity(100, percent);
  frontarmmotorgroup.setMaxTorque(100, percent);
  BackArmMotorGroup.setVelocity(100, percent);
  BackArmMotorGroup.setMaxTorque(100, percent);
  // FrontArmMotor1.stop();
  // FrontArmMotor2.stop();
  wait(0.2, seconds);
  retractguide();
}

void lowerfrontarmfromstackingheight() {
  threadfrontclawcontrol.interrupt();
  if (frontclawcounterstate % 3 == 2) {
    frontclawcounterstate = 0;
    closefinger();
    raisebackarmthread = thread(raisebackarmtobigstandoffystack);
    backarmstatecounter = 2;
    lowerfrontarmfromstandoffheight();
    LeftMotor.setStopping(brake);
    RightMotor.setStopping(brake);
    raisebackarmthread.interrupt();
    isfingeropen = false;
  } else if (!iscornergoalstacking) {
    if (driveforwardstate) {
      frontarmmotorgroup.setStopping(brake);
      drivetrainthread.interrupt();
      LeftMotor.setVelocity(100, percent);
      RightMotor.setVelocity(100, percent);
      LeftMotor.spin(forward);
      RightMotor.spin(forward);
      frontclawcounterstate = 0;
      iscornergoalstacking = false;
      frontarmmotorgroup.setVelocity(100, percent);
      frontarmmotorgroup.setTimeout(2, seconds);
      movefrontclawdown();
      wait(0.29, seconds);
      frontarmmotorgroup.setStopping(hold);
      frontarmmotorgroup.stop();
      while (Controller.ButtonLDown.pressing()) {
        Brain.playSound(siren);
        if (Controller.ButtonRDown.pressing()) {
          Stackingbuttonstate = !Stackingbuttonstate;
          if (!Stackingbuttonstate) {
            frontarmmotorgroup.setVelocity(100, percent);
            frontarmmotorgroup.spinToPosition(155, degrees, true);
            wait(0.2, seconds);
          } else {
            frontarmmotorgroup.setVelocity(100, percent);
            frontarmmotorgroup.spinToPosition(260, degrees, true);
            wait(0.2, seconds);
          }
        }
        printf("in func");
        wait(20, msec);
      }
      frontarmmotorgroup.spinToPosition(155, degrees, true);
      frontclawopen();
      drivetrainthread = thread(splitdrivewithcrawlmode);
      frontarmmotorgroup.setTimeout(2, seconds);
      frontarmmotorgroup.spinToPosition(0, degrees, true);
      frontarmmotorgroup.stop();
      frontarmmotorgroup.setVelocity(100, percent);
      frontclawcounterstate = 0;
      isclawgrabbed = true;
    } else {
      frontclawcounterstate = 0;
      frontarmmotorgroup.setStopping(brake);
      iscornergoalstacking = false;
      frontarmmotorgroup.setVelocity(100, percent);
      frontarmmotorgroup.setTimeout(2, seconds);
      movefrontclawdown();
      wait(0.29, seconds);
      frontarmmotorgroup.setStopping(hold);
      frontarmmotorgroup.stop();
      while (Controller.ButtonLDown.pressing()) {
        Brain.playSound(siren);
        if (Controller.ButtonRDown.pressing()) {
          Stackingbuttonstate = !Stackingbuttonstate;
          if (!Stackingbuttonstate) {
            frontarmmotorgroup.setVelocity(100, percent);
            frontarmmotorgroup.spinToPosition(155, degrees, true);
            wait(0.2, seconds);
          } else {
            frontarmmotorgroup.setVelocity(100, percent);
            frontarmmotorgroup.spinToPosition(260, degrees, true);
            wait(0.2, seconds);
          }
        }
        printf("in func");
        wait(20, msec);
      }
      frontarmmotorgroup.spinToPosition(155, degrees, true);
      frontclawopen();
      frontarmmotorgroup.setTimeout(2, seconds);
      frontarmmotorgroup.spinToPosition(0, degrees, true);
      frontarmmotorgroup.stop();
      frontarmmotorgroup.setVelocity(100, percent);
      frontclawcounterstate = 0;
      isclawgrabbed = true;
    }
    // wait(0.15, seconds);
    // retractclawbalancer();
  } else if (iscornergoalstacking) {
    LeftMotor.setStopping(brake);
    RightMotor.setStopping(brake);
    frontclawleftopen();
    isclawgrabbed = true;
    wait(0.5, seconds);
    // drivetrainthread.interrupt();
    // LeftMotor.setVelocity(100, percent);
    // RightMotor.setVelocity(100, percent);
    // LeftMotor.spin(reverse);
    // RightMotor.spin(reverse);
    // wait(0.75, seconds);
    // drivetrainthread = thread(splitdrivewithcrawlmode);
    iscornergoalstacking = false;
    movefrontclawdown();
    while (frontarmmotorgroup.position(degrees) > 3) {
      wait(20, msec);
    }
    wait(0.35, seconds);
    frontarmmotorgroup.setStopping(brake);
    frontarmmotorgroup.stop();
    isfrontclawup = true;
    frontclawcounterstate = 0;
  }
  isfrontclawdownstate = true;
}

void raisefrontarmtostackingheight() {
  isfrontclawdownstate = false;
  frontarmmotorgroup.setStopping(hold);
  frontclawleftclose();
  wait(0.1, seconds);
  isclawgrabbed = true;
  extendclawbalancer();
  // wait(0.2, seconds);
  movefrontclawup();
  wait(0.15, seconds);
  frontarmmotorgroup.setVelocity(100, percent);
  frontarmmotorgroup.spinToPosition(250, degrees, false);
}

void raisebackarmtonest() {
  spinbackarmup();
  BackArmMotorGroup.setVelocity(50, percent);
  // while (motor1.position(degrees) < 145) {
  //   wait(20, msec);
  // }
  backarmstop();
  BackArmMotorGroup.setVelocity(100, percent);
}

void grabstartingpin() { // minor issue unresolved why is the backarm not
                         // raising for first click then raising to standoff
                         // height
                         // if (!frontclawcounterstate % 3 == 2) {
  startingpingrabstate = !startingpingrabstate;
  if (startingpingrabstate) {
    frontclawopen();
    frontarmmotorgroup.setStopping(hold);
    frontarmmotorgroup.spinToPosition(92, degrees, false);
    retractclawbalancer();
  } else {
    // extendpusher();
    // wait(0.5, seconds);
    // retractpusher();
    extendclawbalancer();
    frontclawleftclose();
    frontarmmotorgroup.spinToPosition(210, degrees, false);
    drivetrainthread.interrupt();
    LeftMotor.setVelocity(100, percent);
    RightMotor.setVelocity(100, percent);
    LeftMotor.spin(reverse);
    RightMotor.spin(reverse);
    drivetrainthread = thread(splitdrivewithcrawlmode);
    isfrontclawup = false;
  }
  // }
}

void grabpins() {
  // wait(0.1, seconds);
  isclawgrabbed = !isclawgrabbed;
  if (isclawgrabbed) {
    frontclawleftopen();
  } else {
    movefrontclawdown();
    frontclawleftclose();
    wait(0.15, seconds);
    frontclawstop();
  }
}
void raisefrontarmtostandoffheight() {
  LeftMotor.setStopping(hold);
  RightMotor.setStopping(hold);
  isfrontclawup = false;
  frontarmmotorgroup.spinToPosition(frontclawstandoffheight, degrees, false);
  extendclawbalancer();
}

void lowerfrontarmfromstandoffheight() {
  killdrivetrain();
  // FrontArmMotor1.setVelocity(100, percent);
  // FrontArmMotor2.setVelocity(100, percent);
  frontarmmotorgroup.setVelocity(100, percent);
  // LeftMotor.setStopping(coast);
  // RightMotor.setStopping(coast);
  // movefrontclawdown();
  // while (FrontArmMotor1.position(degrees) > 235) { // 230
  //   wait(20, msec);
  // }
  // wait(0.15, seconds);
  // FrontArmMotor1.stop();
  // FrontArmMotor2.stop();
  frontclawopen();
  // wait(0.3, seconds);
  // wait(0.15, seconds);
  // movefrontclawdown();
  // wait(0.2, seconds);
  // frontclawstop();
  // wait(0.2, seconds);
  frontarmmotorgroup.spinToPosition(200, degrees, true);
  // FrontArmMotor1.spinToPosition(120, degrees, true);
  // FrontArmMotor2.spinToPosition(120, degrees, true);
  // FrontArmMotor1.setStopping(brake);
  // FrontArmMotor2.setStopping(brake);
  // LeftMotor.setStopping(coast);
  drivetrainthread = thread(splitdrivewithcrawlmode);
  while (BackArmMotorGroup.position(degrees) < 500) {
    wait(20, msec);
  }

  // RightMotor.setStopping(coast);
  frontarmmotorgroup.spinToPosition(5, degrees, true);
  frontarmmotorgroup.setStopping(brake);
  frontarmmotorgroup.stop();
  frontarmmotorgroup.setVelocity(100, percent);
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
  // retractclawbalancer();
}

void stackpinsincornergoal() {
  retractclawbalancer();
  frontclawleftclose();
  wait(0.15, seconds);
  // if (frontarmmotorgroup.position(degrees) < 145) {
  //   movefrontclawup();
  //   // while (frontarmmotorgroup.position(degrees) < 110) {
  //   //   wait(20, msec);
  //   // }
  //   wait(0.5, seconds);
  //   frontarmmotorgroup.setStopping(hold);
  //   frontclawstop();
  // }
  frontarmmotorgroup.setStopping(hold);
  frontarmmotorgroup.setVelocity(50, percent);
  frontarmmotorgroup.spinToPosition(cornergoalstackingheight, degrees, false);
  frontarmmotorgroup.setVelocity(100, percent);
  frontarmmotorgroup.setStopping(hold);

  isfrontclawup = false;
}

void pushercontrol() {
  ispusherextended = !ispusherextended;
  if (ispusherextended) {
    retractpusher();
  } else {
    extendpusher();
  }
}
void stackinginpinmultipress() {
  LeftMotor.setStopping(coast);
  RightMotor.setStopping(coast);
  stackpinsincornergoal();
  // extendclawbalancer();
  Brain.playSound(doorClose);
  isfrontclawup = false;
  isclawgrabbed = true;
  wait(0.75, seconds);
  frontclawcounterstate = 0;
}

void buttonlogic() {
  if (!Controller.ButtonLDown.pressing()) {
    if (Controller.ButtonRDown.pressing()) {
      // LeftMotor.stop();
      // RightMotor.stop();
      raisebackarmthread = thread(fingercontrol);
    }
  }
  if (Controller.ButtonFUp.pressing()) {
    backarmcontrol();
  }

  if (Controller.ButtonR3.pressing()) {
    retractclawbalancer();
  }

  if (Controller.ButtonEUp.pressing()) {
    dumppinsontobeam();
  }

  // if (Controller.ButtonEDown.pressing()) {
  //   grabstartingpin();
  // }

  if (Controller.ButtonLDown.pressing()) {
    frontarmmotorgroup.setStopping(brake);
    LeftMotor.setStopping(brake);
    RightMotor.setStopping(brake);
    isfrontclawup = !isfrontclawup;
    if (isfrontclawup) {
      // retractclawbalancer();
      // wait(0.2, seconds);
      lowerfrontarmfromstackingheight();
      iscornergoalstacking = false;
      threadfrontclawcontrol.interrupt();
    } else {
      iscornergoalstacking = false;
      threadfrontclawcontrol.interrupt();
      raisefrontarmtostackingheight();
    }
  }

  // if (Controller.ButtonL3.pressing()) {
  //   drivetrainthread.interrupt();
  //   LeftMotor.setVelocity(100, percent);
  //   RightMotor.setVelocity(100, percent);
  //   LeftMotor.spin(forward);
  //   RightMotor.spin(forward);
  //   wait(0.65, seconds);
  //   LeftMotor.spin(reverse);
  //   RightMotor.spin(reverse);
  //   wait(0.8, seconds);
  //   drivetrainthread = thread(splitdrivewithcrawlmode);
  // }

  if (Controller.ButtonFDown.pressing()) {
    frontclawcounterstate++;
    if (frontclawcounterstate % 3 == 1) {
      // Brain.playSound(siren2);
      // retractclawbalancer();
      iscornergoalstacking = true;
      // cornergoalstackingheight = cornergoalstackingheight + 5;
      threadfrontclawcontrol = thread(stackinginpinmultipress);
      // wait(0.75, seconds);
      // frontclawcounterstate = 0;
    } else if (frontclawcounterstate % 3 == 2) {
      threadfrontclawcontrol.interrupt();
      iscornergoalstacking = false;
      raisefrontarmtostandoffheight();
    }
  }
  if (!iscornergoalstacking) {
    // if (frontclawcounterstate % 3 == 2) {
    if (!(backarmstatecounter % 3 == 2)) {
      if (Controller.ButtonRUp.pressing()) {
        printf("iscornergoalstacking%d\n", iscornergoalstacking);
        printf("the clawcounterstate is %d\n", frontclawcounterstate);
        // printf("frontclawc%d\n", iscornergoalstacking);
        if (!iscornergoalstacking && !(frontclawcounterstate % 3 == 2)) {
          printf("inside func");
          grabstartingpin();
        }
        // }
      }
    } else if (Controller.ButtonRUp.pressing()) {
      standoffheightstatecounter++;
      if (standoffheightstatecounter % 3 == 1) {
        touchled5.setColor(red);
      } else if (standoffheightstatecounter % 3 == 2) {
        touchled5.setColor(yellow_green);
      } else {
        touchled5.setColor(purple);
      }
      Brain.playSound(siren);
    }
  }

  if (isfrontclawdownstate) {
    if (!iscornergoalstacking) {
      // if (frontclawcounterstate % 3 == 2) {
      if (Controller.ButtonLUp.pressing()) {
        if (!iscornergoalstacking && !(frontclawcounterstate % 3 == 2)) {
          printf("inside func");
          grabpins();
        }
        // }
      }
    }
  }
}

void disconnectionfunc() {
  if (FrontArmMotor1.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("frontarmotor is disconnected port1");
  }
  if (FrontArmMotor2.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("motor2 is disconnected port2");
  }
  if (BackArmMotor1.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("motor6 is disconnected port6");
  }
  if (BackArmMotor2.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("motor7 is disconnected port7");
  }
  if (LeftMotor.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("leftmotor is disconnected port8");
  }
  if (RightMotor.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("rightmotor is disconnected port2");
  }
  if (P1.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("P1 disconnected port4");
  }
  if (P2.installed() == false) {
    Brain.playSound(siren);
    Brain.Screen.print("P1 disconnected port10");
  }
}

void driveforwardcontrolfunc() {
  driveforwardstate = !driveforwardstate;
  Brain.playSound(siren);
  if (driveforwardstate) {
    touchled5.setColor(red);
  } else {
    touchled5.setColor(green);
  }
}

void trianglegoalreset() {
  frontclawcounterstate = 0;
  iscornergoalstacking = false;
  isfrontclawdownstate = true;
  isclawgrabbed = false;
  isfrontclawup = true;
  frontarmmotorgroup.spinToPosition(5, degrees, true);
}

int main() {
  inital();
  pumpon();
  standoffheightstatecounter = 0;
  frontclawstandoffheight = 300;
  drivetrainthread = thread(splitdrivewithcrawlmode);
  eventraisebackarmtonest = event(raisebackarmtonest);
  Controller.ButtonLUp.pressed(buttonlogic);
  Controller.ButtonRDown.pressed(buttonlogic);
  Controller.ButtonRUp.pressed(buttonlogic);
  Controller.ButtonFUp.pressed(buttonlogic);
  Controller.ButtonR3.pressed(buttonlogic);
  // Controller.ButtonR3.pressed(retractguide);
  Controller.ButtonL3.pressed(trianglegoalreset);
  Controller.ButtonEUp.pressed(buttonlogic);
  Controller.ButtonEDown.pressed(buttonlogic);
  Controller.ButtonLDown.pressed(buttonlogic);
  Controller.ButtonFDown.pressed(buttonlogic);
  Brain.playSound(tada);
  touchled5.pressed(driveforwardcontrolfunc);
  eventbackclawgodownforbeam = event(backarmgodownforbeam);
  eventfrontclawgodown = event(movefrontclawdown);
  extendclawbalancer();
  // Run main drive control loop
  while (true) {
    Brain.Screen.setFont(mono15);
    if (Brain.buttonLeft.pressing()) {
      Brain.Screen.clearScreen();
      if (frontclawstandoffheight < 410) {
        frontclawstandoffheight = frontclawstandoffheight + 1;
        Brain.playSound(doorClose);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("standoffH is %d\n", frontclawstandoffheight);
        Brain.Screen.setCursor(1, 2);
        wait(0.2, seconds);
      } else {
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("standoffH is too high");
        Brain.Screen.setCursor(1, 2);
      }
    } else if (Brain.buttonRight.pressing()) {
      Brain.Screen.clearScreen();
      if (frontclawstandoffheight > 280) {
        frontclawstandoffheight = frontclawstandoffheight - 1;
        Brain.playSound(doorClose);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("standoffH is %d\n", frontclawstandoffheight);
        Brain.Screen.setCursor(1, 2);
      } else {
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("standoffH is too low");
        Brain.Screen.setCursor(1, 2);
      }
    }
    disconnectionfunc();
    // printf("positioning of A is %d\n", Controller.AxisA.position());
    // printf("positioning of B is %d\n", Controller.AxisB.position());
    if (iscornergoalstacking || frontclawcounterstate % 3 == 2) {
      // } else {
      if (Controller.ButtonLUp.pressing()) {
        // frontarmmotorgroup.setVelocity(100, percent);
        Dinput = 10;
        frontarmmotorgroup.spinFor(Dinput, degrees);
        cornergoalstackingheight = cornergoalstackingheight + Dinput;
        if (frontclawcounterstate % 3 == 1) {
          // Brain.playSound(siren2);
          // retractclawbalancer();
          iscornergoalstacking = true;
          frontarmmotorgroup.setVelocity(100, percent);
          frontarmmotorgroup.spinFor(Dinput, degrees);
          cornergoalstackingheight = cornergoalstackingheight + Dinput;
          // wait(0.75, seconds);
          // frontclawcounterstate = 0;
        } else if (frontclawcounterstate % 3 == 2) {
          threadfrontclawcontrol.interrupt();
          iscornergoalstacking = false;
          frontarmmotorgroup.spinFor(Dinput, degrees);
          frontclawstandoffheight = frontclawstandoffheight + Dinput;
        }
        Brain.playSound(ratchet2);
      }
      if (Controller.ButtonRUp.pressing()) {
        Dinput = -7;
        frontarmmotorgroup.spinFor(Dinput, degrees);
        cornergoalstackingheight = cornergoalstackingheight + Dinput;
        Brain.playSound(ratchet);
        if (frontclawcounterstate % 3 == 1) {
          // Brain.playSound(siren2);
          // retractclawbalancer();
          iscornergoalstacking = true;
          frontarmmotorgroup.setVelocity(100, percent);
          frontarmmotorgroup.spinFor(Dinput, degrees);
          cornergoalstackingheight = cornergoalstackingheight + Dinput;
          // wait(0.75, seconds);
          // frontclawcounterstate = 0;
        } else if (frontclawcounterstate % 3 == 2) {
          threadfrontclawcontrol.interrupt();
          iscornergoalstacking = false;
          frontarmmotorgroup.spinFor(Dinput, degrees);
          frontclawstandoffheight = frontclawstandoffheight + Dinput;
        }
      }
    }

    // }
    // printf("The front claw position is %.2f\n",
    //        FrontArmMotor1.position(degrees));
    Brain.Screen.setFont(mono15);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("backarm  %.2f\n", BackArmMotor1.position(degrees));
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("frontarmmotor %.2f\n ",
                       FrontArmMotor1.position(degrees));
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("frontarmmotor2 %.2f\n ",
                       FrontArmMotor2.position(degrees));
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("farmgroup%.2f\n ",
                       frontarmmotorgroup.position(degrees));
    // if (!Controller.ButtonLUp.pressing()) {
    //   if (isclawsensorsdetecting) {
    //     if (distancesensorleftclaw.objectDistance(mm) <
    //     distancedetection) {
    //       if (isfrontclawleftopen) {
    //         Brain.playSound(doorClose);
    //         frontclawleftclose();
    //       }
    //     }
    //     if (distancesensorightclaw.objectDistance(mm) <
    //     distancedetection) {
    //       if (isfrontclawrightopen) {
    //         Brain.playSound(doorClose);
    //         frontclawrightclose();
    //       }
    //     }
    //   }
    // }
    wait(20, msec);
  }
}
