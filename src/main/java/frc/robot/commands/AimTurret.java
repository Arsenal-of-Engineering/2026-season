package frc.robot.commands;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.ShuffleboardInfo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SimpleMoveMotor;
import frc.robot.subsystems.drive.Drive;

public class AimTurret extends Command {
  private SimpleMoveMotor m_aimingMotor;
  private SimpleMoveMotor m_angleMotor;
  private double speed = 0;
  private double kAimingDegreesPerRotation;

  public AimTurret(SimpleMoveMotor aimingMotor, SimpleMoveMotor angleMotor, boolean isLeft) {
    kAimingDegreesPerRotation = isLeft ? Constants.kLeftAimingDegreesPerRotation : Constants.kRightAimingDegreesPerRotation;
    m_aimingMotor = aimingMotor;
    m_angleMotor = angleMotor;
    m_angleMotor.initPID(.07, .00005, 0, isLeft);
    m_aimingMotor.initPID(.14, 0, 0, false);
    addRequirements(m_aimingMotor, m_angleMotor);
  }

  /** Sets the target degrees to aim the turret at if it is within the turret's range of motion 
   * @param degrees The target horizontal rotation of the turret
   * @return If the target was successfully changed
  */
  public boolean setTarget(double degrees) {
    double targetDeg = degrees;
    if (targetDeg < -110 || targetDeg > 110)
      return false;
    
    m_aimingMotor.setSetpoint(degrees / kAimingDegreesPerRotation);
    return true;
  }

  public boolean setAngleTarget(double encoderVal) {
    if (encoderVal < 0 || encoderVal > 20)
      return false;
    
    m_angleMotor.setSetpoint(-encoderVal);
    return true;
  }

  public void execute() {
    SmartDashboard.putBoolean("Ready on Aim", m_aimingMotor.isAtSetpoint());
    // double current = m_aimingMotor.getEncoderPosition() * kAimingDegreesPerRotation;

    // speed = MathUtil.clamp(rotatePID.calculate(current, target), -1, 1);
    // m_aimingMotor.set(speed * kAimingDegreesPerRotation / Constants.kLeftAimingDegreesPerRotation);

    // double angleSpeed = MathUtil.clamp(anglePID.calculate(-m_angleMotor.getEncoderPosition(), angleTarget), -.5, .5);

    // SmartDashboard.putNumber("Angle error (" + m_aimingMotor.toString(),  -angleTarget - m_angleMotor.getEncoderPosition());
    // m_angleMotor.set(-angleSpeed);
  }

  public double getSpeed() {
    return speed;
  }
  /**
   * @return the rotation of the turret in degrees (-90,0)
   */
  public double getRotation(){
    return m_aimingMotor.getEncoderPosition() * kAimingDegreesPerRotation;
  }

  @Override
  public void end(boolean interrupted) {
      m_aimingMotor.set(0);
      m_angleMotor.set(0);
  }
}
