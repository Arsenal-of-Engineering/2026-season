package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
   * @param xPos [meters] horizontal (side-to-side) position of the hub <strong>relative to the
   *     robot</strong>
   * @param xVel [m/s] horizontal speed of the hub <strong>relative to the robot</strong>
   * @param yPos [meters] vertical (driver station-to-driver station) position of the hub
   *     <strong>relative to the robot</strong>
   * @param yVel [m/s] vertical speed of the hub <strong>relative to the robot</strong>
   * @return a double array with field relative turret angle [degrees] at index 0 and shoot motor velocity [rpm] at
   *     index 1
   */
  public static double[] calculate(double xPos, double xVel, double yPos, double yVel) {
    // hub position
    xPos = 4.634 - xPos;
    yPos = 4.029 - yPos;
    long t1 = System.nanoTime();
    double a = Math.PI / 2.5;
    double shooterHeight = Units.inchesToMeters(27);
    double goalHeight = 2;
    double l = goalHeight - shooterHeight;


    // non-intertal shot duration calculation
    double guessV = 3 * Math.sqrt(109 * Math.tan(a) / 2) * Math.hypot(xPos, yPos);
    guessV /= 10 * Math.sqrt((shooterHeight - goalHeight) / Math.tan(a) + Math.hypot(xPos, yPos));
    double guessT =
        10.0
            / 981
            * (Math.sqrt(2 * (50 * Math.pow(guessV, 2) - 981 * (goalHeight - shooterHeight)))
                + 10 * guessV);


    double t = solve(guessT, a, xPos, yPos, xVel, yVel, l);
    long t2 = System.nanoTime();
    System.out.println("t = " + t);
    System.out.println((t2 - t1) / 1000000.0 + " ms");


    double angle = Math.atan((yPos + yVel * t) / (xPos + xVel * t));
    if ((xPos + xVel * t) < 0) angle += Math.PI;

    angle *= 180 / Math.PI;

    // calculated initial vertical velocity [m/s] of the ball
    double velocity = 981 * t * t + 200 * (goalHeight - shooterHeight);
    velocity /= 200 * t;
    
    // convert into tangential velocity [m/s]
    velocity /= Math.sin(a);

    // convert m/s to rpm when diameter = 4 inches
    double rpm = velocity / (Math.PI * Units.inchesToMeters(4)) * 60;

    double distance = Math.hypot((yPos + yVel * t), (xPos + xVel * t));
    SmartDashboard.putNumber("distance to hub", distance);
    
    return new double[] {angle, rpm};
  }

  private static double testingAngleToTime(double angle) {
    return 0.0;
  }
  // Function f(x)
  private static double f(
      double x, double a, double xPos, double yPos, double xVel, double yVel, double l) {
    double left = Math.tan(a) * Math.hypot(xPos + xVel * x, yPos + yVel * x);
    double right = 4.905 * Math.pow(x - testingAngleToTime(1), 2) + l;
    return left - right;
  }

  // Numerical derivative f'(x)
  private static double df(
      double x, double a, double xPos, double yPos, double xVel, double yVel, double l) {
    double h = 1e-6;
    return (f(x + h, a, xPos, yPos, xVel, yVel, l) - f(x - h, a, xPos, yPos, xVel, yVel, l))
        / (2 * h);
  }

  private static double solve(
      double initialGuess, double a, double xPos, double yPos, double xVel, double yVel, double l) {

    double x = initialGuess;
    int maxIter = 1000;
    double tol = 1e-8;

    for (int i = 0; i < maxIter; i++) {
      double fx = f(x, a, xPos, yPos, xVel, yVel, l);
      double dfx = df(x, a, xPos, yPos, xVel, yVel, l);

      if (Math.abs(dfx) < 1e-12) {
        throw new RuntimeException("Derivative too small");
      }

      double xNew = x - fx / dfx;

      if (Math.abs(xNew - x) < tol) {
        return xNew;
      }

      x = xNew;
    }

    return x;
  }
}
