package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class MoveArc extends Command {
  private Drive m_drive;
  private boolean right;
  private double xSpeed, ySpeed = 0;

  public MoveArc(Drive drive, boolean isRight) {
    m_drive = drive;
    right = isRight;
  }

  public void initialize() {}

  public void execute() {

    double a = Math.atan(m_drive.getPose().getX() / m_drive.getPose().getY());
    if (m_drive.getPose().getY() < 0) {
      a += Math.PI;
    }
    xSpeed = -Math.sin(a) / 2 * (right ? -1 : 1);
    ySpeed = -Math.cos(a) / 2 * (right ? -1 : 1);
    System.out.println("lemme get uhh.. " + ySpeed + " and a uhm " + xSpeed);
  }

  public double getXSpeed() {
    return xSpeed;
  }

  public double getYSpeed() {
    return ySpeed;
  }

  public boolean isFinished() {
    return false;
  }

  public void end(boolean interrupted) {}
}
