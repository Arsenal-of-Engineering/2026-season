package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class SolveTrajectoryPower {
  /**
   * Newton-Raphson solver for t in these equasions: <br>
   *
   * <pre>
   * physical distance of target varying with time, if the robot moves with <strong>contant velocity</strong>:</p>
   *   hypot(xPos + xVel * t, yPos + yVel * t)
   * ideal ball-to-hub travel distance varying with ball travel time: </p>
   *   (4.905*t^2 + (goalHeight-shooterHeight))/tan(a)
   * </pre>
   *
   * @param xPos [meters] horizontal (side-to-side) position of the hub
   * @param xVel [m/s] horizontal speed of the hub
   * @param yPos [meters] vertical (driver station-to-driver station) position of the hub
   * @param yVel [m/s] vertical speed of the hub
   * @return a double array with field relative turret angle [degrees] at index 0, shoot motor velocity [rpm] at
   *     index 1, and angle motor encoder at index 2
   */
  public static double[] calculate(double xPos, double xVel, double yPos, double yVel, double xTarget, double yTarget, int offsetDirection, double goalHeight) {
    double facing = Math.atan((yTarget - yPos) / (xTarget - xPos));
    double xHub = xTarget + Math.sin(facing) * offsetDirection * Units.inchesToMeters(4);
    double yHub = yTarget + Math.cos(facing) * offsetDirection * Units.inchesToMeters(4);
    System.out.println("-----");
    System.out.println("I'm at "+ xPos + ", " + yPos);
    System.out.println("Let's shoot at " + xHub + ", " + yHub + " im " + offsetDirection);
    
    // hub position
    xPos =  xHub - xPos;
    yPos = yHub - yPos;
    //long t1 = System.nanoTime();
    double a = 66 * Math.PI /180.0;
    double shooterHeight = .65; //meters
    double l = goalHeight - shooterHeight;
    //System.out.println("dist: " + Math.hypot(xPos, yPos));


    // non-intertal shot duration calculation
    double guessV = 3 * Math.sqrt(109 * Math.tan(a) / 2) * Math.hypot(xPos, yPos);
    //System.out.println("V1: " + guessV);
    guessV /= 10 * Math.sqrt((shooterHeight - goalHeight) / Math.tan(a) + Math.hypot(xPos, yPos));
    //System.out.println("V2: " + guessV);
    double guessT =
        10.0
            / 981
            * (Math.sqrt(2 * (50 * Math.pow(guessV, 2) - 981 * (goalHeight - shooterHeight)))
                + 10 * guessV);
  //System.out.println(50 * Math.pow(guessV, 2) - 981 * (goalHeight - shooterHeight));
    
    if (goalHeight == 0){
      return getValues(xPos,  xVel,  yPos,  yVel, guessT);
    }
    double t = solve(guessT, a, xPos, yPos, xVel, yVel, l);
    double[] vals = getValues(xPos,  xVel,  yPos,  yVel, t);
    System.out.println("right so that's an angle of " + vals[0]);
    System.out.println("-----");
    return vals;
  }

  private static double[] getValues(double xPos, double xVel, double yPos, double yVel, double t){
    double angle = Math.atan((yPos + yVel * t) / (xPos + xVel * t));
      if ((xPos + xVel * t) < 0) angle += Math.PI;

      angle *= 180 / Math.PI;

      double distance = Math.hypot((yPos + yVel * t), (xPos + xVel * t));
      SmartDashboard.putNumber("distance to hub", distance);

      // regression line from google sheets
      //double rpm = MathUtil.clamp(459 * distance + 2153, 0, 6767);
      double rpm = MathUtil.clamp(4171 - 1326 * distance + 407 * Math.pow(distance,2), 0, 6767);
      
      // (angle motor)
      double churro = Math.max(3.36 * distance - 3.63, 0);

      return new double[] {angle, rpm, churro};
  }

  

  // Function f(x)
  private static double f(
      double x, double a, double xPos, double yPos, double xVel, double yVel, double l) {
    a = getAngle(Math.hypot(xPos + xVel * x, yPos + yVel * x));
    double left = Math.tan(a) * Math.hypot(xPos + xVel * x, yPos + yVel * x);
    double right = 4.905 * Math.pow(x, 2) + l;
    return left - right;
  }

  // Numerical derivative f'(x)
  private static double df(
      double x, double a, double xPos, double yPos, double xVel, double yVel, double l) {
    double h = 1e-6;
    return (f(x + h, a, xPos, yPos, xVel, yVel, l) - f(x - h, a, xPos, yPos, xVel, yVel, l))
        / (2 * h);
  }

  private static double getAngle(double distanceToHub){
    // regression line from google sheets
    double churro = MathUtil.clamp(0.191 + -3.89 * distanceToHub + 3.63 * Math.pow(distanceToHub,2) + -0.484 * Math.pow(distanceToHub,3), 0,20);
  
    // interp between 0 = 66 degrees and 20 = 50 degrees
    return (66 - 16*(churro/20)) * Math.PI / 180.0;
  }

  private static double solve(
      double initialGuess, double a, double xPos, double yPos, double xVel, double yVel, double l) {

    double x = initialGuess;
    int maxIter = 1000;
    double tol = 1e-8;

    for (int i = 0; i < maxIter; i++) {
      double fx = f(x, a, xPos, yPos, xVel, yVel, l);
      double dfx = df(x, a, xPos, yPos, xVel, yVel, l);

      // if (Math.abs(dfx) < 1e-12) {
      //   throw new RuntimeException("Derivative too small");
      // }

      double xNew = x - fx / dfx;

      if (Math.abs(xNew - x) < tol) {
        return xNew;
      }

      x = xNew;
    }

    return x;
  }
}
