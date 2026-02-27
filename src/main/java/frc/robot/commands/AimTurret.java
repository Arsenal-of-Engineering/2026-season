package frc.robot.commands;

import edu.wpi.first.epilogue.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AddressableLED;
// import frc.robot.ShuffleboardInfo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SimpleMoveMotor;
import frc.robot.subsystems.drive.Drive;

public class AimTurret extends Command {
  private PIDController rotatePID =
      new PIDController(
          .02, 0.0, 0.00, .02);
  private SimpleMoveMotor m_aimingMotor;
  private PIDController anglePID =
      new PIDController(
          .003, 0.0, 0.00, .02);
  private SimpleMoveMotor m_angleMotor;
  /** Goal encoder value */
  private double target = 0;
  private double angleTarget = 0;
  private double speed = 0;

  public AimTurret(SimpleMoveMotor aimingMotor, SimpleMoveMotor angleMotor) {
    m_aimingMotor = aimingMotor;
    m_angleMotor = angleMotor;
    addRequirements(m_aimingMotor, m_angleMotor);
  }

  /** Sets the target degrees to aim the turret at if it is within the turret's range of motion 
   * @param degrees The target horizontal rotation of the turret
   * @return If the target was successfully changed
  */
  public boolean setTarget(double degrees) {
    double targetEncoder = degrees;
    if (targetEncoder < -85 || targetEncoder > 0)
      return false;
    
    target = targetEncoder;
    return true;
  }

  public boolean setAngleTarget(double encoderVal) {
    double targetEncoder = encoderVal;
    if (targetEncoder < 0 || targetEncoder > 20)
      return false;
    
    angleTarget = targetEncoder;
    return true;
  }

  public void execute() {
    System.out.println("im 6 feet from the edge");
    double current = m_aimingMotor.getEncoderPosition() * Constants.kAimingDegreesPerRotation;

    speed = MathUtil.clamp(rotatePID.calculate(current, target), -1, 1);

    m_aimingMotor.set(speed);

    double angleSpeed = MathUtil.clamp(anglePID.calculate(m_angleMotor.getEncoderPosition(), angleTarget), -.5, .5);

    m_angleMotor.set(angleSpeed);
  }

  public double getSpeed() {
    return speed;
  }
  /**
   * @return the rotation of the turret in degrees (-90,0)
   */
  public double getRotation(){
    return m_aimingMotor.getEncoderPosition() * Constants.kAimingDegreesPerRotation;
  }

  @Override
  public void end(boolean interrupted) {
      m_aimingMotor.set(0);
      m_angleMotor.set(0);
  }
}
