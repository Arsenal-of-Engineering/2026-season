package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class MoveArc extends Command {
  private Drive m_drive;
  private Rotate m_rotate;
  private boolean right;
  private double xSpeed, ySpeed = 0;

  public MoveArc(Drive drive, Rotate m_Rotate, boolean isRight) {
    m_drive = drive;
    m_rotate = m_Rotate;
    right = isRight;
  }

  public void initialize() {
    // CommandScheduler.getInstance().schedule(m_rotate);
  }

  public void execute() {

    double a = Math.atan(m_drive.getPose().getX() / m_drive.getPose().getY());
    // a += (right ? -1 : 1) * Math.PI / 180.0;
    if (m_drive.getPose().getY() < 0) {
      a += Math.PI;
    }
    m_rotate.setTarget(a);
    xSpeed = -Math.sin(a) / 3 * (right ? -1 : 1);
    ySpeed = -Math.cos(a) / 3 * (right ? -1 : 1);
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

  public void end(boolean interrupted) {
    // CommandScheduler.getInstance().cancel(m_rotate);
  }
}
