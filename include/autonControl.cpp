#include "vex.h"


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
  //20 x 20 pitch (0-20)
  //Square worth 4 coord
  //sqaure 600mm
  //1 coord 150mm same as base radius
  public:
    Coord ROBOT = Coord(0, 0);
    double rAngle = 0;

    const Coord BIG_MID = Coord(10, 10);
    const Coord LEFT_MID = Coord(4, 10);
    const Coord RIGHT_MID = Coord(17, 10);
    const Coord ALLIANCE_PLAT = Coord(15, 0);
    const Coord ALLAIANCE_FLOOR = Coord(0, 4);

    void moveToCoords(vex::drivetrain drv, Coord dest, double vel){
      double distToMove = calcDistance(dest);
      double nang = calcAngle(dest);
      double ang = nang;

      // Big Block of angle checking code, pls don't touch otherwise i'll get mad.
      if (ROBOT.x > dest.x && ROBOT.y < dest.y) {
        ang = ang - (ang*2);
        // if (rAngle != 0) {
        //   ang = 360 + ang - rAngle;
        // }
      }
      else if (ROBOT.x < dest.x && ROBOT.y > dest.y) {
        ang = ang - (ang*2);        
        // if (rAngle != 0) {
        //   ang = 360 + ang - rAngle;
        // }
      }
      else if (ROBOT.x <= dest.x && ROBOT.y <= dest.y) {
        // if (rAngle != 0) {
        //   ang = ang - rAngle;
        // }
      }
      else if (ROBOT.x >= dest.x && ROBOT.y >= dest.y) {       
        // if (rAngle != 0) {
        //   ang = ang - rAngle;
        // }
      }
      
      // I like to move it move it
      drv.turnFor(1, rotationUnits::deg, vel, velocityUnits::pct);
      drv.driveFor(distToMove, distanceUnits::cm, vel, velocityUnits::pct);

      // Update robot position
      ROBOT = dest;
      if (nang < 0) {
        rAngle = 360 + nang;
      }
      else {
        rAngle = nang;
      }
    }

    FieldMap(Coord nRobot, double nAngle){
      ROBOT = nRobot;
      rAngle = nAngle;
    };

  private:
    double calcDistance(Coord dest){
      double dist;

      double X = ROBOT.x - dest.x;
      double Y = ROBOT.y - dest.y;
      double x = pow(X, 2);
      double y = pow(Y, 2);

      dist = sqrt(x + y);
      if (dist < 0){
        dist = dist + (dist*-2);
      }
      else {
        dist = dist - (dist*2);
      }
      dist = dist * 12;

      return dist;
    }

    double calcAngle(Coord dest){
      double ang;
      
      double X = ROBOT.x - dest.x;
      double Y = ROBOT.y - dest.y;

      double angleRadian = atan2(X, Y);
      ang = angleRadian * (180 / M_PI);

      return ang;
    }
};