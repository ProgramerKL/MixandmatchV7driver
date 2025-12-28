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
motor motor2 = motor(PORT2, false);
motor motor8 = motor(PORT8, true);
motor FrontArmMotor1 = motor(PORT4, true);
motor FrontArmMotor2 = motor(PORT10, false);
pneumatic P1 = pneumatic(PORT11);
// from front view left
pneumatic P2 = pneumatic(PORT12);
pneumatic P3 = pneumatic(PORT6);
float Rmotorspeed, Lmotorspeed, Rightstick, Leftstick, deadband = 10;
bool crawlmodestate;
bool isfrontclawrightopen;
bool isfrontclawleftopen;
bool isfingeropen;
bool isbackarmup;
bool ispusherextended;
int distancedetection = 80;
int frontclawcounterstate;
int backarmstatecounter;
bool isstandoffgoalstacking;
bool isfrontclawup;
bool startingpingrabstate;
bool touchledstate;
bool isclawgrabbed = true;
int crawlspeed = 35;
void lowerfromstandoffgoal();
void splitdrivewithcrawlmode();
event eventfrontclawgodownforbeam = event();
thread drivetrainthread = thread();
event eventraisebackarmtogroundystack = event();
event eventraisebackarmtoministandoffstack = event();
event eventraisebackarmtobigstandoffystack = event();
event eventraisebackarmtonest = event();
event eventfrontclawgodown = event();
thread raisebackarmthread = thread();
// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

void killdrivetrain() {
  drivetrainthread.interrupt();
  LeftMotor.stop();
  RightMotor.stop();
  LeftMotor.setVelocity(0, percent);
  RightMotor.setVelocity(0, percent);
}

void touchledcolourselection() {
  if (touchledstate) {
    touchled5.setColor(green);
  } else {
    touchled5.setColor(red);
  }
}

void deployfrontguide() { P3.extend(cylinder2); }

void retractfrontguide() { P3.retract(cylinder2); }

void deployguide() {
  // P3.extend(cylinder1);
  P3.extend(cylinder1);
}

void retractguide() {
  // P3.retract(cylinder1);
  P3.retract(cylinder1);
}

void pumpon() {
  P1.pumpOn();
  P2.pumpOn();
  P3.pumpOn();
}
void extendclawbalancer() { P2.extend(cylinder1); }
void retractclawbalancer() { P2.retract(cylinder1); }

void extendpusher() { P1.extend(cylinder2); }
void retractpusher() { P1.retract(cylinder2); }

void frontclawleftclose() {
  P1.retract(cylinder1);
  isfrontclawleftopen = false;
}

void frontclawleftopen() {
  P1.extend(cylinder1);
  isfrontclawleftopen = true;
}

void frontclawrightclose() {
  P1.retract(cylinder2);
  isfrontclawrightopen = false;
}
void frontclawrightopen() {
  P1.extend(cylinder2);
  isfrontclawrightopen = true;
}

void frontclawopen() {
  frontclawleftopen();
  frontclawrightopen();
}

void closefinger() {
  P2.extend(cylinder2);
  isfingeropen = false;
}

void openfinger() {
  P2.retract(cylinder2);
  isfingeropen = true;
}

void spinbackarmup() {
  BackArmMotor1.setStopping(hold);
  BackArmMotor2.setStopping(hold);
  BackArmMotor1.spin(forward);
  BackArmMotor2.spin(forward);
}

void spinbackarmdown() {
  BackArmMotor1.setStopping(coast);
  BackArmMotor2.setStopping(coast);
  BackArmMotor1.spin(reverse);
  BackArmMotor2.spin(reverse);
}

void backarmstop() {
  BackArmMotor1.stop();
  BackArmMotor2.stop();
}

void movefrontclawup() {
  isfrontclawup = true;
  FrontArmMotor1.spin(forward);
  FrontArmMotor2.spin(forward);
}

void movefrontclawdown() {
  isfrontclawup = false;
  FrontArmMotor1.spin(reverse);
  FrontArmMotor2.spin(reverse);
}

void frontclawstop() {
  FrontArmMotor1.stop();
  FrontArmMotor2.stop();
}

void fingercontrol() {
  // retractguide();
  if (isfingeropen) {
    spinbackarmdown();
    drivetrainthread.interrupt();
    LeftMotor.setVelocity(100, percent);
    RightMotor.setVelocity(100, percent);
    RightMotor.spin(reverse);
    LeftMotor.spin(reverse);
    wait(100, msec);
    closefinger();
    wait(0.2, seconds);
    drivetrainthread = thread(splitdrivewithcrawlmode);
    // eventraisebackarmtonest.broadcast();
    // wait(0.2, seconds);
    spinbackarmup();
    while (BackArmMotor1.position(degrees) < 65) {
      wait(20, msec);
    }
    BackArmMotor1.setStopping(hold);
    BackArmMotor2.setStopping(hold);
    backarmstop();
  } else {
    BackArmMotor1.setStopping(coast);
    BackArmMotor2.setStopping(coast);
    BackArmMotor1.setVelocity(50, percent);
    BackArmMotor2.setVelocity(50, percent);
    spinbackarmdown();
    wait(0.4, seconds);
    backarmstop();
    BackArmMotor1.setVelocity(100, percent);
    BackArmMotor2.setVelocity(100, percent);
    openfinger();
    if (BackArmMotor1.position(degrees) > 250) {
      spinbackarmup();
      wait(0.4, seconds);
      backarmstop();
      drivetrainthread.interrupt();
      LeftMotor.setVelocity(100, percent);
      RightMotor.setVelocity(100, percent);
      RightMotor.spin(forward);
      LeftMotor.spin(forward);
      wait(0.4, seconds);
      drivetrainthread = thread(splitdrivewithcrawlmode);
    }
    spinbackarmdown();
    while (BackArmMotor1.position(degrees) > 20) {
      wait(20, msec);
    }
    BackArmMotor1.setStopping(coast);
    BackArmMotor2.setStopping(coast);
    backarmstop();
    backarmstatecounter = 0;
  }
}

void frontarmgodownforbeam() {
  spinbackarmdown();
  wait(0.5, seconds);
  spinbackarmup();
  while (BackArmMotor1.position(degrees) < 75) {
    wait(20, msec);
  }
  BackArmMotor1.setStopping(hold);
  BackArmMotor2.setStopping(hold);
  backarmstop();
}

void dumppinsontobeam() {
  frontclawleftclose();
  frontclawrightclose();
  isclawgrabbed = false;
  wait(0.1, seconds);
  retractfrontguide();
  wait(0.1, seconds);
  closefinger();
  eventfrontclawgodownforbeam.broadcast();
  movefrontclawup();
  while (FrontArmMotor1.position(degrees) < 500) { // 625
    wait(20, msec);
  }
  FrontArmMotor1.setStopping(brake);
  FrontArmMotor2.setStopping(brake);
  FrontArmMotor1.setVelocity(60, percent);
  FrontArmMotor2.setVelocity(60, percent);
  while (FrontArmMotor1.position(degrees) < 720) {
    wait(20, msec);
  }
  frontclawopen();
  frontclawstop();
  FrontArmMotor1.setVelocity(100, percent);
  FrontArmMotor2.setVelocity(100, percent);
  wait(0.2, seconds);
  eventfrontclawgodown.broadcast();
  extendclawbalancer();
  wait(0.2, seconds);
  while (FrontArmMotor2.position(degrees) > 5) {
    wait(20, msec);
  }
  BackArmMotor1.setStopping(hold);
  BackArmMotor2.setStopping(hold);
  backarmstop();
  FrontArmMotor1.setStopping(coast);
  FrontArmMotor2.setStopping(coast);
  frontclawstop();
  eventraisebackarmtonest.broadcast();
  retractclawbalancer();
  isclawgrabbed = false;
}

// ==============================================================================
// ARM FUNCTIONS
// ==============================================================================

void lowerbackarmfromstandoff() {
  spinbackarmdown();
  while (BackArmMotor1.position(degrees) > 1080) {
    wait(20, msec);
  }
  openfinger();
  while (BackArmMotor1.position(degrees) > 10) {
    wait(20, msec);
  }
  backarmstop();
}
void lowerbackarmfromstandoffwithoutrelease() {
  BackArmMotor1.setVelocity(60, percent);
  BackArmMotor2.setVelocity(60, percent);
  spinbackarmdown();
  while (BackArmMotor1.position(degrees) > 10) {
    wait(20, msec);
  }
  backarmstop();
  BackArmMotor1.setVelocity(100, percent);
  BackArmMotor2.setVelocity(100, percent);
}
void raisebackarmtogroundystack() {
  if (BackArmMotor1.position(degrees) > 300) {
    spinbackarmdown();
    while (BackArmMotor1.position(degrees) > 300) {
      wait(20, msec);
    }
    BackArmMotor1.setStopping(hold);
    BackArmMotor2.setStopping(hold);
    backarmstop();
    deployguide();
  } else {
    spinbackarmup();
    while (BackArmMotor1.position(degrees) < 300) {
      wait(20, msec);
    }
    backarmstop();
    deployguide();
  }
}

void raisebackarmtoministandoffstack() {
  if (BackArmMotor1.position(degrees) > 400) {
    spinbackarmdown();
    while (BackArmMotor1.position(degrees) > 400) {
      wait(20, msec);
    }
    BackArmMotor1.setStopping(hold);
    BackArmMotor2.setStopping(hold);
    backarmstop();
  } else {
    spinbackarmup();
    while (BackArmMotor1.position(degrees) < 400) {
      wait(20, msec);
    }
    backarmstop();
  }
}

void raisebackarmtobigstandoffystack() {
  if (BackArmMotor1.position(degrees) > 500) {
    spinbackarmdown();
    while (BackArmMotor1.position(degrees) > 750) {
      wait(20, msec);
    }
    BackArmMotor1.setStopping(hold);
    BackArmMotor2.setStopping(hold);
    wait(0.3, seconds);
    backarmstop();
  } else {
    spinbackarmup();
    while (BackArmMotor1.position(degrees) < 500) {
      wait(20, msec);
    }
    wait(0.3, seconds);
    backarmstop();
  }
}

void backarmcontrol() {
  backarmstatecounter++;
  raisebackarmthread.interrupt();
  if (backarmstatecounter % 4 == 1) { // raise to 91 height
    closefinger();
    raisebackarmthread = thread(raisebackarmtogroundystack);
    touchled5.setColor(green);
  } else if (backarmstatecounter % 4 == 2) { // raise to 110 height
    raisebackarmthread = thread(raisebackarmtoministandoffstack);
    touchled5.setColor(yellow);
    // retractguide();
    closefinger();
  } else if (backarmstatecounter % 4 == 3) { // raise to 121 height
    raisebackarmthread = thread(raisebackarmtobigstandoffystack);
    touchled5.setColor(red);
  } else if (backarmstatecounter % 4 == 0) {
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
  P3.extend(cylinder1);
  P3.extend(cylinder2);
  isbackarmup = false;
  isfrontclawleftopen = true;
  isfrontclawrightopen = true;
  isfingeropen = true;
  openfinger();
  frontclawopen();
  BackArmMotor1.setStopping(hold);
  BackArmMotor2.setStopping(hold);
  LeftMotor.setMaxTorque(100, percent);
  RightMotor.setMaxTorque(100, percent);
  FrontArmMotor1.setVelocity(100, percent);
  FrontArmMotor2.setVelocity(100, percent);
  FrontArmMotor1.setMaxTorque(100, percent);
  FrontArmMotor2.setMaxTorque(100, percent);
  BackArmMotor1.setVelocity(100, percent);
  BackArmMotor2.setVelocity(100, percent);
  BackArmMotor1.setMaxTorque(100, percent);
  BackArmMotor2.setMaxTorque(100, percent);
  spinbackarmdown();
  wait(0.2, seconds);
  backarmstop();
  FrontArmMotor1.setPosition(0, degrees);
  FrontArmMotor2.setPosition(0, degrees);
  retractfrontguide();
  wait(0.2, seconds);
  // retractguide();
}

void stackpins() {
  FrontArmMotor2.setStopping(hold);
  FrontArmMotor1.setStopping(hold);
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
  if (isfrontclawup) {
    extendclawbalancer();
    movefrontclawdown();
    retractfrontguide();
    wait(0.1, seconds);
    while (FrontArmMotor1.position(degrees) > 145) {
      wait(20, msec);
    }
    frontclawopen();
    while (FrontArmMotor1.position(degrees) > 5) {
      wait(20, msec);
    }
    frontclawstop();
    frontclawcounterstate = 0;
    // wait(1, seconds);
  } else {
    frontclawleftclose();
    frontclawrightclose();
    isclawgrabbed = true;
    extendclawbalancer();
    deployfrontguide();
    movefrontclawup();
    while (FrontArmMotor1.position(degrees) < 195) {
      wait(20, msec);
    }
    frontclawstop();
  }
}

void raisebackarmtonest() {
  spinbackarmup();
  BackArmMotor1.setVelocity(50, percent);
  BackArmMotor2.setVelocity(50, percent);
  // while (motor1.position(degrees) < 145) {
  //   wait(20, msec);
  // }
  backarmstop();
  BackArmMotor1.setVelocity(100, percent);
  BackArmMotor2.setVelocity(100, percent);
}

void grabstartingpin() {
  startingpingrabstate = !startingpingrabstate;
  if (startingpingrabstate) {
    frontclawopen();
    FrontArmMotor1.setStopping(hold);
    FrontArmMotor2.setStopping(hold);
    movefrontclawup();
    while (FrontArmMotor1.position(degrees) < 160) {
      wait(20, msec);
    }
    frontclawstop();
  } else {
    // extendpusher();
    // wait(0.5, seconds);
    // retractpusher();
    frontclawleftclose();
    frontclawrightclose();
    movefrontclawup();
    while (FrontArmMotor1.position(degrees) < 160) {
      wait(20, msec);
    }
    drivetrainthread.interrupt();
    LeftMotor.setVelocity(100, percent);
    RightMotor.setVelocity(100, percent);
    LeftMotor.spin(reverse);
    RightMotor.spin(reverse);
    wait(0.25, seconds);
    frontclawstop();
    drivetrainthread = thread(splitdrivewithcrawlmode);
    // movefrontclawdown();
    // while (motor6.position(degrees) > 175) {
    //   wait(20, msec);
    // }
    // isfrontclawup = true;
    // frontclawstop();
    // extendpusher();
    // wait(0.25, seconds);
    // retractpusher();
  }
}

void grabpins() {
  isclawgrabbed = !isclawgrabbed;
  if (isclawgrabbed) {
    frontclawleftopen();
    frontclawrightopen();
  } else {
    frontclawleftclose();
    frontclawrightclose();
  }
}

void stackpinsontostandoff() {
  LeftMotor.setStopping(hold);
  RightMotor.setStopping(hold);
  isstandoffgoalstacking = !isstandoffgoalstacking;
  if (isstandoffgoalstacking) {
    movefrontclawup();
    LeftMotor.setStopping(brake);
    RightMotor.setStopping(brake);
    extendclawbalancer();
    while (FrontArmMotor1.position(degrees) < 320) { // 300
      wait(20, msec);
    }
    FrontArmMotor2.setStopping(hold);
    FrontArmMotor1.setStopping(hold);
    frontclawstop();
    wait(0.1, seconds);
    // extendpusher();
  } else {
    killdrivetrain();
    FrontArmMotor1.setVelocity(30, percent);
    FrontArmMotor2.setVelocity(30, percent);
    movefrontclawdown();
    // drivetrainthread. = thread(splitdrivewithcrawlmode);
    while (FrontArmMotor1.position(degrees) > 295) {
      wait(20, msec);
    }
    frontclawopen();
    // FrontArmMotor1.setVelocity(0, percent);
    // FrontArmMotor2.setVelocity(0, percent);
    // wait(0.2, seconds);
    FrontArmMotor1.setVelocity(-80, percent);
    FrontArmMotor2.setVelocity(-80, percent);
    while (FrontArmMotor1.position(degrees) > 5) {
      wait(20, msec);
    }
    FrontArmMotor1.setStopping(hold);
    FrontArmMotor2.setStopping(hold);
    frontclawstop();
    LeftMotor.setStopping(coast);
    RightMotor.setStopping(coast);
    FrontArmMotor1.setVelocity(100, percent);
    FrontArmMotor2.setVelocity(100, percent);
    // retractclawbalancer();
    frontclawcounterstate = 0;
  }
  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
  drivetrainthread = thread(splitdrivewithcrawlmode);
  wait(0.5, seconds);
}

void stackpinsincornergoal() {
  movefrontclawup();
  while (FrontArmMotor1.position(degrees) < 115) {
    wait(20, msec);
  }
  FrontArmMotor2.setStopping(hold);
  FrontArmMotor1.setStopping(hold);
  frontclawstop();
  frontclawcounterstate = 0;
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
  if (Controller.ButtonLUp.pressing()) {
    grabpins();
  }
  if (Controller.ButtonRDown.pressing()) {
    fingercontrol();
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
  if (Controller.ButtonEDown.pressing()) {
    grabstartingpin();
  }
  if (Controller.ButtonLDown.pressing()) {
    stackpins();
  }
  if (Controller.ButtonFDown.pressing()) {
    frontclawcounterstate++;
    if (frontclawcounterstate % 3 == 1) {
      stackpins();
    } else if (frontclawcounterstate % 3 == 2) {
      stackpinsincornergoal();
    } else if (frontclawcounterstate % 3 == 0) {
      frontclawleftclose();
      frontclawrightclose();
      isclawgrabbed = true;
      // wait(0.15, seconds);
      stackpinsontostandoff();
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

int main() {
  inital();
  pumpon();
  drivetrainthread = thread(splitdrivewithcrawlmode);
  eventraisebackarmtonest = event(raisebackarmtonest);
  Controller.ButtonLUp.pressed(buttonlogic);
  Controller.ButtonRDown.pressed(buttonlogic);
  Controller.ButtonFUp.pressed(buttonlogic);
  Controller.ButtonR3.pressed(buttonlogic);
  // Controller.ButtonR3.pressed(retractguide);
  // Controller.ButtonL3.pressed(buttonlogic);
  Controller.ButtonEUp.pressed(buttonlogic);
  Controller.ButtonEDown.pressed(buttonlogic);
  Controller.ButtonLDown.pressed(buttonlogic);
  Controller.ButtonFDown.pressed(buttonlogic);
  Brain.playSound(tada);
  eventfrontclawgodownforbeam = event(frontarmgodownforbeam);
  eventfrontclawgodown = event(movefrontclawdown);
  // Run main drive control loop
  while (true) {
    touchledcolourselection();
    Brain.Screen.setFont(mono15);
    if (Brain.buttonLeft.pressing()) {
      Brain.Screen.clearScreen();
      if (crawlspeed < 95) {
        Brain.playSound(doorClose);
        crawlspeed = crawlspeed + 5;
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("crawlspeed, %d\n", crawlspeed);
        Brain.Screen.setCursor(1, 2);
        wait(0.2, seconds);
      } else {
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("crawlspeed too high");
        Brain.Screen.setCursor(1, 2);
        wait(0.2, seconds);
      }
    } else if (Brain.buttonRight.pressing()) {
      Brain.Screen.clearScreen();
      if (crawlspeed > 5) {
        Brain.playSound(ratchet);
        crawlspeed = crawlspeed - 5;
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("crawlspeed, %d\n", crawlspeed);
        Brain.Screen.setCursor(1, 2);
        wait(0.2, seconds);
      } else {
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("crawlspeed too low");
        wait(0.2, seconds);
      }
    }
    disconnectionfunc();
    // printf("positioning of A is %d\n", Controller.AxisA.position());
    // printf("positioning of B is %d\n", Controller.AxisB.position());
    if (Controller.ButtonRUp.pressing()) {
      touchledstate = false;
      crawlmodestate = true;
      Brain.playSound(siren);
      touchled5.setBlink(yellow, 0.2);
      wait(0.15, seconds);
    } else {
      crawlmodestate = false;
      wait(0.1, seconds);
      touchled5.setColor(red);
    }

    // if (!Controller.ButtonLUp.pressing()) {
    //   if (isclawsensorsdetecting) {
    //     if (distancesensorleftclaw.objectDistance(mm) < distancedetection) {
    //       if (isfrontclawleftopen) {
    //         Brain.playSound(doorClose);
    //         frontclawleftclose();
    //       }
    //     }
    //     if (distancesensorightclaw.objectDistance(mm) < distancedetection) {
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
