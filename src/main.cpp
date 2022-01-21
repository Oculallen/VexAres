/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    19, 18, 20, 17  
// Controller1          controller                    
// gripper              motor         12              
// liftgrip             motor         15              
// lift                 motor         14              
// minilift             motor         16              
// Inertial10           inertial      10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
#include <cmath>

competition Competition;

class Coord{
  public:
    int x;
    int y;

    Coord(int nx, int ny){
      x = nx;
      y = ny;
    }
};

class FieldMap{
  //20 x 20 pitch
  //Square worth 4 coord
  //sqaure 600mm
  //1 coord 150mm same as base radius
  public:
    Coord ROBOT = Coord(0, 0);
    double rAngle = 0;

    const Coord BIG_MID = Coord(10, 10);
    const Coord LEFT_MID = Coord(4, 10);
    const Coord RIGHT_MID = Coord(16, 10);
    const Coord ALLIANCE_PLAT = Coord(15, 0);
    const Coord ALLAIANCE_FLOOR = Coord(0, 4);

    void moveToCoords(vex::drivetrain drv, Coord dest, double vel){
      double distToMove = calcDistance(dest);
      double angleToTurn = calcAngle(dest);
      drv.turnFor(angleToTurn, rotationUnits::deg, vel, velocityUnits::pct);
      drv.driveFor(distToMove, distanceUnits::mm, vel, velocityUnits::pct);
      ROBOT = dest;
    }

    FieldMap(Coord nRobot, double nAngle){
      ROBOT = nRobot;
      rAngle = nAngle;
    };

  private:
    double calcDistance(Coord dest){
      double dist;

      dist = pow((dest.x - ROBOT.x), 2) + pow((dest.y - ROBOT.y), 2);
      dist = sqrt(dist);
      

      return dist;
    }

    double calcAngle(Coord dest){
      double ang;
      double dist;
      double ydist = dest.y-ROBOT.y;

      dist = calcDistance(dest);
      ang = cos(ydist/dist);
      ang = ang * 30;

      if (ROBOT.x > dest.x) {
        ang = ang - (ang*2);
      }

      return ang;
    }
};

// a big ugly block of driving constants
// dw mos it doesn't look too bad
const double THROTTLE_VEL = 50;
const double BOOST_VEL = 100;
const double TURN_VEL = 0.25;
const double POWERSLIDE_VEL = 0.50;
const double DRIFT_MULTIPLIER = 1.20;
const double MIN_LIFT_VEL = 50;
const double MAX_LIFT_VEL = 100;
const double LIFT_STICKMIN = 5;
const double LIFT_STICKMAX = 100;
const double GRIP_VEL = 50;
const double MINILIFT_VEL = 100;
const double LIFTGRIP_VEL = 100;
const double COORD_SPACE = 150  ;
// Morg needs to do testing on this tonight
const double MINILIFT_DOWN = 100;
const double LIFT_UP = 100;
const double LIFTGRIP_DOWN = 100;
const double LIFTGRIP_UP = 20;
const double GRIP_DOWN = 60;
// Scale constant for setting motors
const int SCALE = 120;


double linterp(double y0, double y1, double x, double x0, double x1) {
  return (y0 * (x1 - x) + y1 * (x - x0)) / (x1 - x0);
}

void coastdrive() {
  LeftDriveSmart.setStopping(coast);
  RightDriveSmart.setStopping(coast);
}
void holddrive() {
  LeftDriveSmart.setStopping(hold);
  RightDriveSmart.setStopping(hold);
}


bool reverseTurn = false;
bool flipDrive = false;
void rlDrivePlus(int stick, bool btnThrottle, bool btnReverse, bool btnBoost, bool btnPowerslide, bool btnFlipOn, bool btnFlipOff) {
  // toggle reverse driving
  if(btnFlipOn) flipDrive = true;
  if(btnFlipOff) flipDrive = false;

  // calculate drive velocity
  // reverse turn depending on last direction driven (forward takes priority)
  double driveVel = 0;
  if(btnReverse) {
    reverseTurn = true;
    if(btnBoost) driveVel -= BOOST_VEL;
    else driveVel -= THROTTLE_VEL;
  }
  if(btnThrottle) {
    reverseTurn = false;
    if(btnBoost) driveVel += BOOST_VEL;
    else driveVel += THROTTLE_VEL;
  }

  // calculate turn velocity and drift multiplier
  double turnVel, backWheelMultiplier;
  if(btnPowerslide) {
    turnVel = stick * POWERSLIDE_VEL; // powerslide
    backWheelMultiplier = DRIFT_MULTIPLIER;
  } else {
    turnVel = stick * TURN_VEL; // normal turn
    backWheelMultiplier = 1;
  }

  // find left and right side speeds
  double leftSpeed = 0;
  double rightSpeed = 0;
  if(reverseTurn) {
    // reverse turning
    leftSpeed = driveVel - turnVel;
    rightSpeed = driveVel + turnVel;
  } else {
    leftSpeed = driveVel + turnVel;
    rightSpeed = driveVel - turnVel;
  }

  // set motor velocities (with drift in theory)
  // assume A motors (1, 3) are front and B motors (2, 4) are back
  if(flipDrive) {
    rightMotorB.setVelocity(-leftSpeed, percent);
    rightMotorA.setVelocity(-leftSpeed * backWheelMultiplier, percent);
    leftMotorB.setVelocity(-rightSpeed, percent);
    leftMotorA.setVelocity(-rightSpeed * backWheelMultiplier, percent);
  }
  else {
    leftMotorA.setVelocity(leftSpeed, percent);
    leftMotorB.setVelocity(leftSpeed * backWheelMultiplier, percent);
    rightMotorA.setVelocity(rightSpeed, percent);
    rightMotorB.setVelocity(rightSpeed * backWheelMultiplier, percent);
  }

  // spin motors
  leftMotorA.spin(forward);
  leftMotorB.spin(forward);
  rightMotorA.spin(forward);
  rightMotorB.spin(forward);
}


void liftcontrol(int stickPos) {
  if(abs(stickPos) < LIFT_STICKMIN) {
    lift.stop(hold);
  }
  else {
    double liftVel = linterp(MIN_LIFT_VEL, MAX_LIFT_VEL, abs(stickPos), LIFT_STICKMIN, LIFT_STICKMAX);
    lift.setVelocity(liftVel, percent);
    if(stickPos > 0) {
      lift.spin(forward);
    } else {
      lift.spin(reverse);
    }
  }
}


void genericMotorControl(vex::motor& mtr, bool btnUp, bool btnDown) {
  // new, improved, flexible, and still deals with double button presses!
  // feat: some funny c++ arithmetic trick with booleans and also object pass-by-reference which I don't actually understand
  switch(btnUp - btnDown){
    case 1:
      mtr.spin(forward);
      break;
    case -1:
      mtr.spin(reverse);
      break;
    case 0:
    default:
      mtr.stop(hold);
      break;
  }
}


void setMotor(vex::motor& mtr, int input) {
  mtr.spin(fwd, input*SCALE, voltageUnits::mV);
}

void setMotorPos(vex::motor& mtr, int pos, double speed) {
  mtr.startRotateTo(pos, rotationUnits::deg, speed, velocityUnits::pct);
}


// DOGO inspired motor reset so the robot knows where everything is
void motorSetup(){
  bool RESET = true;
  bool set_grip = false;
  bool mini_zero = false;
  bool lift_zero = false;
  bool liftgrip_zero = false;

  // Setting all motors apart from gripper moving
  // Since gripper is in a bit of a awkward position to move immediatly before the MOGO
  setMotor(minilift, 50); // set_minilift(50);
  setMotor(lift, -20); // set_lift(-20);
  setMotor(liftgrip, -40); // set_liftgrip(-40);

  // Reset main motors to their backstop
  wait(100, timeUnits::msec);
  while (RESET){
    if (minilift.velocity(percentUnits::pct) < 0.8) {
      setMotor(minilift, 0); // set_minilift(0);
      mini_zero = true;
      minilift.resetPosition();
    }

    if (lift.velocity(percentUnits::pct) < 0.8) {
      setMotor(lift, 0); // set_lift(0);
      lift_zero = true;
      lift.resetPosition();
    }

    if (liftgrip.velocity(percentUnits::pct) < 0.8) {
      setMotor(liftgrip, 0); // set_liftgrip(0);
      liftgrip_zero = true;
      liftgrip.resetPosition();
    }

    if (mini_zero && lift_zero && liftgrip_zero) {
      RESET = false;
    }
  }

  // Set Gripper position after init setup
  setMotor(gripper, -10); // set_gripper(-10);
  wait(50, msec);
  while (set_grip) {
    if (gripper.velocity(percentUnits::pct) < 0.8) {
      setMotor(gripper, 0); // set_gripper(0);
      set_grip = false;
      gripper.resetPosition();
    }
  }

  // Nudge section to make sure nothing collides with anything nasty (mostly that darn side gripper)
  setMotorPos(gripper, 5, GRIP_VEL); // setGripPos(5, GRIP_VEL);
  setMotorPos(lift, -2, MIN_LIFT_VEL); // setLiftPos(-2, MIN_LIFT_VEL);
  setMotorPos(minilift, 10, MINILIFT_VEL);
  gripper.resetPosition();
  lift.resetPosition();
  minilift.resetPosition();
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  gripper.setVelocity(GRIP_VEL, percent);
  minilift.setVelocity(MINILIFT_VEL, percent);
  liftgrip.setVelocity(LIFTGRIP_VEL, percent);
  //motorSetup();
}


void auton(void) {
  holddrive();
}

/// Left side of pitch
// - Moves forward with MOGO to get right mid
// - Moves back to get blue plat
///
void autonSeqOne(void){
  FieldMap map(Coord(20, 0), 0);
  minilift.spinFor(1500, rotationUnits::deg);
  
  map.moveToCoords(Drivetrain, map.RIGHT_MID, 70);
}

void usercontrol(void) {
  coastdrive();

  while(true) {
    rlDrivePlus(
      Controller1.Axis4.position(), // stick
      Controller1.ButtonR2.pressing(), // throttle
      Controller1.ButtonL2.pressing(), // reverse
      Controller1.ButtonR1.pressing(), // boost
      Controller1.ButtonL1.pressing(), // powerslide
      Controller1.ButtonDown.pressing(), // flipped driving on
      Controller1.ButtonUp.pressing() // flipped driving off
    );

    liftcontrol(Controller1.Axis2.position());

    genericMotorControl(gripper, Controller1.ButtonRight.pressing(), Controller1.ButtonLeft.pressing());
    genericMotorControl(minilift, Controller1.ButtonB.pressing(), Controller1.ButtonA.pressing());
    genericMotorControl(liftgrip, Controller1.ButtonY.pressing(), Controller1.ButtonX.pressing());

    wait(50, msec);
  }
}


int main() {
  Competition.autonomous(autonSeqOne);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}