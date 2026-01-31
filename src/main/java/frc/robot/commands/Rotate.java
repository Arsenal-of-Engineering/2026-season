package frc.robot.commands;

import edu.wpi.first.epilogue.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import frc.robot.ShuffleboardInfo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class Rotate extends Command {
  private ProfiledPIDController profiledRotatePID =
      new ProfiledPIDController(
          .015, 0.015, 0.00005, new TrapezoidProfile.Constraints(1, .25), .02);
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 2.05, .28);
  private Drive m_drive;
  private double target = 0;
  private double speed = 0;

  public Rotate(Drive drive) {
    m_drive = drive;
  }

  public void setTarget(double degrees) {
    target = degrees;
  }

  public void execute() {
    double head = m_drive.getRotation().getDegrees();
    double del = target - head;
    if (del < -180) head -= 360;
    if (del > 180) head += 360;

    speed = MathUtil.clamp(profiledRotatePID.calculate(head, target), -1, 1);
  }

  public double getSpeed() {
    return speed;
  }
}
