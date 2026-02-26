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
motor LeftMotor = motor(PORT11, false);
motor RightMotor = motor(PORT5, true);
// motor motor2 = motor(PORT2, true);
touchled touchled5 = touchled(PORT9);
motor BackArmMotor1 = motor(PORT12, false);
motor BackArmMotor2 = motor(PORT6, true);
motor_group BackArmMotorGroup = motor_group(BackArmMotor1, BackArmMotor2);
motor FrontArmMotor1 = motor(PORT10, false);
motor FrontArmMotor2 = motor(PORT2, true);
motor_group frontarmmotorgroup = motor_group(FrontArmMotor1, FrontArmMotor2);
pneumatic P1 = pneumatic(PORT3);
// from front view left
pneumatic P2 = pneumatic(PORT4);
float Rmotorspeed, Lmotorspeed, Rightstick, Leftstick, deadband = 10;
float Dinput;
bool driveforwardstate = false;
bool iscornergoalstacking;
bool Trianglemovingstate;
bool isfingeropen;
bool istrianglestacking;
bool Stackingbuttonstate = false;
int standoffheightstatecounter;
bool ispusherextended;
bool isguidedeployed;
int frontclawstandoffheight;
bool isintriangle1;
bool isintriangel2;
int cornergoalstackingheight = 130;
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
int startingpingrabcounter;
bool isclawgrabbed = true;
bool clawstartingcontrol;
int crawlspeed = 35;
void lowerfromstandoffgoal();
void splitdrivewithcrawlmode();
void stack91();
void trianglegoalreset();
void stack84();
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
  isguidedeployed = true;
  // P3.extend(cylinder1);
}

void retractguide() {
  P1.retract(cylinder1);
  isguidedeployed = false;
  // P3.retract(cylinder1);
}

void pumpon() {
  P1.pumpOn();
  P2.pumpOn();
  // P3.pumpOn();
}
void extendclawbalancer() { P2.retract(cylinder1); }
void retractclawbalancer() { P2.extend(cylinder1); }

void extendpusher() { P1.extend(cylinder2); }
void retractpusher() { P1.retract(cylinder2); }

void frontclawleftclose() { P1.retract(cylinder2); }

void frontclawleftopen() { P1.extend(cylinder2); }

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
    spinbackarmdown();
    // wait(0.1, seconds);
    closefinger();
    wait(0.25, seconds);
    spinbackarmup();
    while (BackArmMotorGroup.position(degrees) < 80) {
      if (Controller.ButtonEUp.pressing()) {
        break;
      }
      wait(20, msec);
    }
    BackArmMotorGroup.setStopping(hold);
    if (!Controller.ButtonEUp.pressing()) {
      backarmstop();
    }
  } else {
    BackArmMotorGroup.stop();
    if (backarmstatecounter % 3 == 1) {
      stack91();
    } else if (backarmstatecounter % 3 == 2) {
      if (standoffheightstatecounter % 3 == 0) {
        BackArmMotorGroup.spinToPosition(473, degrees, true);
        while (Controller.ButtonRDown.pressing()) {
          Brain.playSound(siren);
          if (Controller.AxisD.position() > 80) {
            BackArmMotorGroup.setVelocity(45, percent);
            BackArmMotorGroup.spinToPosition(545, degrees, true);
          } else if (Controller.AxisD.position() < -80) {
            BackArmMotorGroup.setVelocity(45, percent);
            BackArmMotorGroup.spinToPosition(500, degrees, true);
          } else {
            BackArmMotorGroup.setStopping(hold);
            BackArmMotorGroup.stop();
            BackArmMotorGroup.setVelocity(100, percent);
          }
          wait(20, msec);
          BackArmMotorGroup.stop();
        }
        stack121();
      } else if (standoffheightstatecounter % 3 == 1) {
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
      LeftMotor.setVelocity(0, percent);
      RightMotor.setVelocity(0, percent);
      BackArmMotorGroup.stop();
      BackArmMotorGroup.setVelocity(100, percent);
      spinbackarmdown();
      while (BackArmMotorGroup.position(degrees) > 20) {
        wait(20, msec);
      }
      BackArmMotorGroup.setStopping(coast);
      BackArmMotorGroup.setVelocity(100, percent);
      backarmstop();
      openfinger();
      // wait(0.15, seconds);
      // spinbackarmup();
      // if (BackArmMotorGroup.position(degrees) > 100) {
      //   while (BackArmMotorGroup.position(degrees) < 170) {
      //     wait(20, msec);
      //   }
      // }
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
      backarmstatecounter = 0;
    }
  }
}

void backarmgodownforbeam() {
  spinbackarmdown();
  wait(0.45, seconds);
  backarmstop();
  wait(1.2, seconds);
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
  if (isclawgrabbed) {
    frontclawleftclose();
    wait(0.1, seconds);
  }
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
  while (frontarmmotorgroup.position(degrees) < 715) { // 740
    wait(20, msec);
  }
  wait(0.1, seconds);
  frontclawopen();
  isclawgrabbed = true;
  frontclawstop();
  frontarmmotorgroup.setVelocity(100, percent);
  eventfrontclawgodown.broadcast();
  while (frontarmmotorgroup.position(degrees) > 15) {
    wait(20, msec);
  }
  extendclawbalancer();
  frontarmmotorgroup.setStopping(brake);
  frontclawleftopen();
  iscornergoalstacking = false;
  isfrontclawup = false;
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
  while (BackArmMotorGroup.position(degrees) > 100) {
    wait(20, msec);
  }
  backarmstop();
  BackArmMotorGroup.setVelocity(100, percent);
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
    deployguide();
    BackArmMotorGroup.setStopping(hold);
    backarmstop();
  } else {
    spinbackarmup();
    while (BackArmMotorGroup.position(degrees) < 440) {
      wait(20, msec);
    }
    deployguide();
    backarmstop();
  }
}

void raisebackarmtobigstandoffystack() {
  standoffheightstatecounter = 0;
  spinbackarmup();
  while (BackArmMotorGroup.position(degrees) < 555) { // 455
    wait(20, msec);
  }
  retractguide();
  backarmstop();
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
  wait(0.59, seconds);
  deployguide();
  drivetrainthread = thread(splitdrivewithcrawlmode);
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(60, percent);
  spinbackarmdown();
  wait(0.65, seconds);
  BackArmMotorGroup.stop();
  BackArmMotorGroup.setVelocity(100, percent);
  while (Controller.ButtonRDown.pressing()) {
    Brain.playSound(siren);
    if (Controller.AxisD.position() > 30) {
      BackArmMotorGroup.setVelocity(60, percent);
      BackArmMotorGroup.spin(forward);
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
  backarmstatecounter = 0;
  openfinger();
  retractguide();
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
}

void stack121() {
  // LeftMotor.stop();
  // RightMotor.stop();
  BackArmMotorGroup.setVelocity(30, percent);
  spinbackarmdown();
  while (BackArmMotorGroup.position(degrees) > 480) {
    wait(20, msec);
  }
  BackArmMotorGroup.stop();
  BackArmMotorGroup.setVelocity(100, percent);
  openfinger();
  isclawgrabbed = true;
  backarmstatecounter = 0;
  wait(2, seconds);
  backarmstatecounter = 0;
  wait(0.1, seconds);
  // drivetrainthread.interrupt();
  // LeftMotor.setVelocity(100, percent);
  // RightMotor.setVelocity(100, percent);
  // RightMotor.spin(forward);
  // LeftMotor.spin(forward);
  // wait(0.5, seconds);
  // drivetrainthread = thread(splitdrivewithcrawlmode);
  spinbackarmdown();
  while (BackArmMotorGroup.position(degrees) > 100) {
    wait(20, msec);
  }
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(100, percent);
  backarmstop();
}

void stack110() {
  drivetrainthread.interrupt();
  LeftMotor.setVelocity(30, percent);
  RightMotor.setVelocity(30, percent);
  LeftMotor.spin(forward);
  RightMotor.spin(forward);
  wait(0.445, seconds);
  deployguide();
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
      BackArmMotorGroup.spinToPosition(455, degrees, true);
    } else if (Controller.AxisD.position() < -30) {
      BackArmMotorGroup.setVelocity(60, percent);
      BackArmMotorGroup.spinToPosition(412, degrees, true);
    } else {
      BackArmMotorGroup.setStopping(hold);
      BackArmMotorGroup.stop();
      BackArmMotorGroup.setVelocity(100, percent);
    }
    wait(30, msec);
    BackArmMotorGroup.stop();
  }
  openfinger();
  backarmstatecounter = 0;
  wait(0.2, seconds);
  retractguide();
  spinbackarmup();
  if (BackArmMotorGroup.position(degrees) > 380) {
    while (BackArmMotorGroup.position(degrees) < 440) {
      wait(20, msec);
    }
  }
  wait(0.5, seconds);
  backarmstop();
  wait(0.25, seconds);
  spinbackarmdown();
  while (BackArmMotorGroup.position(degrees) > 20) {
    wait(20, msec);
  }
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(100, percent);
  backarmstop();
}

void stack91() {
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(30, percent);
  spinbackarmdown();
  wait(0.43, seconds);
  BackArmMotorGroup.stop();
  BackArmMotorGroup.setVelocity(100, percent);
  deployguide();
  while (Controller.ButtonRDown.pressing()) {
    Brain.playSound(siren);
    if (Controller.AxisD.position() > 30) {
      BackArmMotorGroup.setVelocity(60, percent);
      BackArmMotorGroup.spinToPosition(350, degrees, true);
    } else if (Controller.AxisD.position() < -30) {
      BackArmMotorGroup.setVelocity(60, percent);
      BackArmMotorGroup.spinToPosition(215, degrees, true);
    } else {
      BackArmMotorGroup.setStopping(hold);
      BackArmMotorGroup.stop();
      BackArmMotorGroup.setVelocity(100, percent);
    }
    wait(30, msec);
    BackArmMotorGroup.stop();
  }
  openfinger();
  backarmstatecounter = 0;
  wait(0.45, seconds);
  spinbackarmup();
  wait(0.35, seconds);
  backarmstop();
  retractguide();
  wait(0.2, seconds);
  // wait(1.5, seconds);
  spinbackarmdown();
  while (BackArmMotorGroup.position(degrees) > 20) {
    wait(20, msec);
  }
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(100, percent);
  backarmstop();
}

void stack91onstandoff() {
  drivetrainthread.interrupt();
  LeftMotor.setVelocity(30, percent);
  RightMotor.setVelocity(30, percent);
  LeftMotor.spin(forward);
  RightMotor.spin(forward);
  wait(0.45, seconds);
  drivetrainthread = thread(splitdrivewithcrawlmode);
  BackArmMotorGroup.setStopping(coast);
  BackArmMotorGroup.setVelocity(30, percent);
  spinbackarmdown();
  wait(0.54, seconds);
  BackArmMotorGroup.stop();
  BackArmMotorGroup.setVelocity(100, percent);
  wait(0.15, seconds);
  openfinger();
  spinbackarmup();
  if (BackArmMotorGroup.position(degrees) > 320) {
    while (BackArmMotorGroup.position(degrees) < 380) {
      wait(20, msec);
    }
  }
  wait(0.2, seconds);
  backarmstop();
  if (BackArmMotorGroup.position(degrees) > 100) {
    // spinbackarmup();
    // wait(0.4, seconds);
    // backarmstop();
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

void backarmcontrol() {
  backarmstatecounter++;
  raisebackarmthread.interrupt();
  if (backarmstatecounter % 3 == 1) { // raise to 91 height
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
    if (backarmstatecounter % 3 == 1) {
      C_position = C_position * 0.6;
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
  // P3.extend(cylinder1);
  isfingeropen = true;
  isfrontclawup = false;
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
    if (driveforwardstate) {
      frontclawcounterstate = 0;
      closefinger();
      retractguide();
      raisebackarmthread = thread(raisebackarmtobigstandoffystack);
      backarmstatecounter = 2;
      lowerfrontarmfromstandoffheight();
      LeftMotor.setStopping(brake);
      RightMotor.setStopping(brake);
    } else {
      frontclawcounterstate = 0;
      retractguide();
      lowerfrontarmfromstandoffheight();
    }
  } else if (!iscornergoalstacking) {
    startingpingrabcounter = 0;
    frontclawcounterstate = 0;
    frontarmmotorgroup.setStopping(brake);
    iscornergoalstacking = false;
    frontarmmotorgroup.setVelocity(80, percent);
    frontarmmotorgroup.setTimeout(2, seconds);
    movefrontclawdown();
    wait(0.39, seconds);
    frontarmmotorgroup.setStopping(hold);
    frontarmmotorgroup.stop();
    Stackingbuttonstate = false;
    while (Controller.ButtonLDown.pressing()) {
      printf("in func\n");
      while (!Controller.ButtonRDown.pressing()) {
        if (!Controller.ButtonLDown.pressing()) {
          break;
        } else {
          wait(20, msec);
        }
      }
      Brain.playSound(ratchet2);
      if (Controller.ButtonRDown.pressing()) {
        Stackingbuttonstate = !Stackingbuttonstate;
        if (!Stackingbuttonstate) {
          frontarmmotorgroup.setVelocity(100, percent);
          Brain.playSound(alarm);
          frontarmmotorgroup.spinToPosition(130, degrees, false);
        } else {
          frontarmmotorgroup.setVelocity(100, percent);
          Brain.playSound(doorClose);
          frontarmmotorgroup.spinToPosition(240, degrees, false);
        }
      }
      wait(0.3, seconds);
    }
    frontclawopen();
    isclawgrabbed = true;
    frontarmmotorgroup.setVelocity(100, percent);
    frontarmmotorgroup.spinToPosition(0, degrees, true);
    frontarmmotorgroup.setVelocity(0, percent);
    frontclawcounterstate = 0;
    isfrontclawup = false;
    printf("done func\n");
  } else if (iscornergoalstacking) {
    // movefrontclawdown();
    // wait(0.3, seconds);
    frontclawstop();
    LeftMotor.setStopping(brake);
    RightMotor.setStopping(brake);
    if (isintriangel2) {
      retractclawbalancer();
      frontclawleftopen();
      isclawgrabbed = true;
      wait(1, seconds);
      retractguide();
      iscornergoalstacking = false;
      movefrontclawdown();
      while (frontarmmotorgroup.position(degrees) > -5) {
        wait(20, msec);
      }
      frontarmmotorgroup.setStopping(brake);
      frontarmmotorgroup.stop();
      isfrontclawup = false;
      frontclawcounterstate = 0;
      threadfrontclawcontrol.interrupt();
      extendclawbalancer();
    } else {
      extendclawbalancer();
      isclawgrabbed = true;
      retractguide();
      wait(0.3, seconds);
      iscornergoalstacking = false;
      frontarmmotorgroup.setVelocity(100, percent);
      movefrontclawdown();
      wait(0.3, seconds); // 0.13
      frontclawleftopen();
      while (frontarmmotorgroup.position(degrees) > -5) {
        wait(20, msec);
      }
      frontarmmotorgroup.setStopping(coast);
      frontarmmotorgroup.stop();
      isfrontclawup = false;
      frontclawcounterstate = 0;
      threadfrontclawcontrol.interrupt();
      extendclawbalancer();
      printf("infunc\n");
    }
  }
  isintriangle1 = false;
  isintriangel2 = false;
}

void raisefrontarmtostackingheight() {
  frontarmmotorgroup.setStopping(hold);
  retractguide();
  if (isclawgrabbed) {
    frontclawleftclose();
    wait(0.1, seconds);
  }
  isclawgrabbed = true;
  isfrontclawup = true;
  extendclawbalancer();
  // wait(0.2, seconds);
  frontarmmotorgroup.setVelocity(100, percent);
  frontarmmotorgroup.spinToPosition(285, degrees, true);
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
  startingpingrabcounter = startingpingrabcounter + 1;
  if (startingpingrabcounter % 3 == 1) {
    // drivetrainthread = thread(splitdrivewithcrawlmode);
    printf("in func first\n");
    frontarmmotorgroup.setStopping(hold);
    frontarmmotorgroup.stop();
    deployguide();
    frontclawopen();
    frontarmmotorgroup.setVelocity(100, percent);
    frontarmmotorgroup.setStopping(hold);
    retractclawbalancer();
    frontarmmotorgroup.spinToPosition(102.5, degrees, true);
    frontarmmotorgroup.setStopping(coast);
    frontarmmotorgroup.stop();
  } else if (startingpingrabcounter % 3 == 2) {
    printf("in func second\n");
    retractguide();
    extendclawbalancer();
    frontarmmotorgroup.setVelocity(100, percent);
    frontclawleftclose();
    frontarmmotorgroup.spinToPosition(325, degrees, false);
    wait(0.22, seconds);
    frontarmmotorgroup.spinToPosition(210, degrees, false);
    isfrontclawup = true;
    if (BackArmMotorGroup.position(degrees) > 300) {
      lowerbackarmfromstandoff();
    }
  } else {
    printf("in func third\n");
    trianglegoalreset();
  }
  // }
}

void grabpins() {
  // wait(0.1, seconds);
  isclawgrabbed = !isclawgrabbed;
  if (isclawgrabbed) {
    frontclawleftopen();
  } else {
    frontclawleftclose();
  }
  wait(0.1, seconds);
}
void raisefrontarmtostandoffheight() {
  LeftMotor.setStopping(hold);
  RightMotor.setStopping(hold);
  isfrontclawup = true;
  frontarmmotorgroup.setVelocity(100, percent);
  frontarmmotorgroup.spinToPosition(frontclawstandoffheight, degrees, false);
  extendclawbalancer();
}

void lowerfrontarmfromstandoffheight() {
  killdrivetrain();
  frontarmmotorgroup.setVelocity(86, percent);
  frontclawopen();
  isclawgrabbed = true;
  frontarmmotorgroup.spinToPosition(240, degrees, true);
  drivetrainthread = thread(splitdrivewithcrawlmode);
  frontarmmotorgroup.spinToPosition(5, degrees, true);
  frontarmmotorgroup.setStopping(brake);
  frontarmmotorgroup.stop();
  frontarmmotorgroup.setVelocity(100, percent);
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
}

void Triangle1() {
  if (isclawgrabbed) {
    if (!isguidedeployed) {
      frontarmmotorgroup.spinToPosition(0, degrees, true);
    }
    frontclawleftclose();
    wait(0.15, seconds);
  }
  deployguide();
  if (!isintriangel2) {
    frontarmmotorgroup.setStopping(hold);
    frontarmmotorgroup.setVelocity(100, percent);
    LeftMotor.setStopping(coast);
    RightMotor.setStopping(coast);
    frontarmmotorgroup.spin(forward);
    wait(0.4, seconds); // 0.4
    frontarmmotorgroup.setVelocity(50, percent);
    frontarmmotorgroup.spin(reverse);
    wait(0.5, seconds); // 0.4
    frontarmmotorgroup.setStopping(coast);
    frontarmmotorgroup.stop();
  }
  isintriangle1 = true;
  isintriangel2 = false;
  isfrontclawup = true;
  isclawgrabbed = true;
}

void Triangle2() {
  if (isclawgrabbed) {
    if (!isguidedeployed) {
      frontarmmotorgroup.spinToPosition(0, degrees, true);
    }
    frontclawleftclose();
    wait(0.15, seconds);
  }
  deployguide();
  retractclawbalancer();
  if (!isintriangle1) {
    frontarmmotorgroup.setStopping(coast);
    frontarmmotorgroup.setVelocity(100, percent);
    LeftMotor.setStopping(coast);
    RightMotor.setStopping(coast);
    frontarmmotorgroup.spin(forward);
    wait(0.4, seconds);
    frontarmmotorgroup.setVelocity(50, percent);
    frontarmmotorgroup.spin(reverse);
    wait(0.5, seconds);
    frontarmmotorgroup.setStopping(coast);
    frontarmmotorgroup.stop();
  }
  isfrontclawup = true;
  isclawgrabbed = true;
  isintriangle1 = false;
  isintriangel2 = true;
}

void pushercontrol() {
  ispusherextended = !ispusherextended;
  if (ispusherextended) {
    retractpusher();
  } else {
    extendpusher();
  }
}

void buttonlogic() {
  if (!Controller.ButtonRDown.pressing()) {
    if (Controller.ButtonLDown.pressing()) {
      frontarmmotorgroup.setStopping(brake);
      LeftMotor.setStopping(brake);
      RightMotor.setStopping(brake);
      if (isfrontclawup) {
        Trianglemovingstate = false;
        lowerfrontarmfromstackingheight();
        isfrontclawup = false;
        iscornergoalstacking = false;
      } else {
        iscornergoalstacking = false;
        printf("raising frontarm\n");
        raisefrontarmtostackingheight();
        isfrontclawup = true;
      }
    }
  }

  if (!Controller.ButtonLDown.pressing()) {
    if (Controller.ButtonRDown.pressing()) {
      raisebackarmthread = thread(fingercontrol);
    }
  }

  if (Controller.ButtonFUp.pressing()) {
    backarmcontrol();
  }

  if (Controller.ButtonR3.pressing()) {
    extendclawbalancer();
  }

  if (Controller.ButtonEUp.pressing()) {
    dumppinsontobeam();
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
  if (Controller.ButtonEDown.pressing()) {
    istrianglestacking = !istrianglestacking;
    if (istrianglestacking) {
      frontclawcounterstate = 1;
      Trianglemovingstate = false;
      Brain.playSound(doorClose);
      extendclawbalancer();
      threadfrontclawcontrol = thread(Triangle1);
      iscornergoalstacking = true;
    } else {
      frontclawcounterstate = 1;
      Trianglemovingstate = true;
      retractclawbalancer();
      threadfrontclawcontrol = thread(Triangle2);
      isfrontclawup = false;
      iscornergoalstacking = true;
    }
  }

  if (Controller.ButtonFDown.pressing()) {
    if (!isfrontclawup) {
      frontclawcounterstate = 2;
      retractguide();
      frontclawleftclose();
      wait(0.2, seconds);
      Brain.playSound(tada);
      threadfrontclawcontrol.interrupt();
      iscornergoalstacking = false;
      raisefrontarmtostandoffheight();
    }
  }

  if (Controller.ButtonRUp.pressing()) {
    printf("frontclawstate is %d\n", isfrontclawup);
    printf("frontclawstate is %d\n", frontclawcounterstate);
    // if (isfrontclawup && startingpingrabcounter % 3 == 0 &&
    //     !(frontclawcounterstate == 2)) {
    //   trianglegoalreset();
    // } else {
    if (!(backarmstatecounter % 3 == 2)) {
      raisebackarmthread.interrupt();
      BackArmMotorGroup.stop();
      // printf("iscornergoalstacking%d\n", iscornergoalstacking);
      // printf("the clawcounterstate is %d\n", frontclawcounterstate);
      if (!iscornergoalstacking && !(frontclawcounterstate % 3 == 2)) {
        // printf("inside func");
        grabstartingpin();
      }
      // }
    } else if (Controller.ButtonRUp.pressing()) {
      standoffheightstatecounter++;
      if (standoffheightstatecounter % 3 == 0) {
        touchled5.setColor(red);
      } else if (standoffheightstatecounter % 3 == 1) {
        touchled5.setColor(yellow_green);
      } else {
        touchled5.setColor(purple);
      }
      Brain.playSound(siren);
    }
    // }
  }

  if (!isfrontclawup) {
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
    touchled5.setBlink(blue, 0.2);
    Brain.playSound(siren);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("frontarmM port4");
    Brain.Screen.setPenColor(white);
  }
  if (FrontArmMotor2.installed() == false) {
    touchled5.setBlink(blue, 0.2);
    Brain.playSound(siren);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("FrontarmM1 port10");
    Brain.Screen.setPenColor(white);
  }
  if (BackArmMotor1.installed() == false) {
    touchled5.setBlink(blue, 0.2);
    Brain.playSound(siren);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("backarmM port2");
    Brain.Screen.setPenColor(white);
  }
  if (BackArmMotor2.installed() == false) {
    touchled5.setBlink(blue, 0.2);
    Brain.playSound(siren);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("backarmM port8");
    Brain.Screen.setPenColor(white);
  }
  if (LeftMotor.installed() == false) {
    touchled5.setBlink(blue, 0.2);
    Brain.playSound(siren);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("leftmotor port9");
    Brain.Screen.setPenColor(white);
  }
  if (RightMotor.installed() == false) {
    touchled5.setBlink(blue, 0.2);
    Brain.playSound(siren);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("rightmotor port3");
    Brain.Screen.setPenColor(white);
  }
  if (P1.installed() == false) {
    touchled5.setBlink(blue, 0.2);
    Brain.playSound(siren);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("P1 port11");
    Brain.Screen.setPenColor(white);
  }
  if (P2.installed() == false) {
    touchled5.setBlink(blue, 0.2);
    Brain.playSound(siren);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("P2 port12");
    Brain.Screen.setPenColor(white);
  }
  if (touchled5.installed() == false) {
    touchled5.setBlink(blue, 0.2);
    Brain.playSound(siren);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("touchled port5");
    Brain.Screen.setPenColor(white);
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
  isclawgrabbed = true;
  isfrontclawup = false;
  istrianglestacking = false;
  Trianglemovingstate = false;
  retractguide();
  frontarmmotorgroup.spinToPosition(5, degrees, true);
  frontclawleftopen();
}

int main() {
  inital();
  pumpon();
  standoffheightstatecounter = 0;
  frontclawstandoffheight = 320;
  drivetrainthread = thread(splitdrivewithcrawlmode);
  eventraisebackarmtonest = event(raisebackarmtonest);
  Controller.ButtonLUp.pressed(buttonlogic);
  Controller.ButtonRDown.pressed(buttonlogic);
  Controller.ButtonRUp.pressed(buttonlogic);
  Controller.ButtonFUp.pressed(buttonlogic);
  Controller.ButtonR3.pressed(buttonlogic);
  // Controller.ButtonR3.pressed(retractguide);
  Controller.ButtonEUp.pressed(buttonlogic);
  Controller.ButtonEDown.pressed(buttonlogic);
  Controller.ButtonLDown.pressed(buttonlogic);
  Controller.ButtonFDown.pressed(buttonlogic);
  Brain.playSound(tada);
  touchled5.pressed(driveforwardcontrolfunc);
  eventfrontclawgodown = event(movefrontclawdown);
  eventbackclawgodownforbeam = event(backarmgodownforbeam);
  extendclawbalancer();
  // Run main drive control loop
  while (true) {
    Brain.Screen.setFont(mono15);
    disconnectionfunc();
    // printf("positioning of A is %d\n", Controller.AxisA.position());
    // printf("positioning of B is %d\n", Controller.AxisB.position());
    if (frontclawcounterstate % 3 == 2) {
      // } else {
      if (Controller.AxisD.position() > 80) {
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
        } else if (frontclawcounterstate % 3 == 2) {
          threadfrontclawcontrol.interrupt();
          iscornergoalstacking = false;
          frontarmmotorgroup.spinFor(Dinput, degrees);
          frontclawstandoffheight = frontclawstandoffheight + Dinput;
        }
        Brain.playSound(ratchet2);
      }
      if (Controller.AxisD.position() < -80) {
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
    //        frontarmmotorgroup.position(degrees));
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
