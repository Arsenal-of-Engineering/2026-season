package frc.robot.commands;

import edu.wpi.first.epilogue.*;
import edu.wpi.first.math.MathUtil;
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
  private ProfiledPIDController profiledRotatePID =
      new ProfiledPIDController(
          .015, 0.015, 0.00005, new TrapezoidProfile.Constraints(1, .25), .02);
  private SimpleMoveMotor m_aimingMotor;
  /** Goal encoder value */
  private double target = 0;
  private double speed = 0;

  public AimTurret(SimpleMoveMotor aimingMotor) {
    m_aimingMotor = aimingMotor;
    addRequirements(m_aimingMotor);
  }

  /** Sets the target degrees to aim the turret at if it is within the turret's range of motion 
   * @param degrees The target horizontal rotation of the turret
   * @return If the target was successfully changed
  */
  public boolean setTarget(double degrees) {
    double targetEncoder = degrees / Constants.kAimingDegreesPerRotation;
    if (m_aimingMotor.getEncoderRange() != null){
      if (targetEncoder < m_aimingMotor.getEncoderRange()[0] || targetEncoder > m_aimingMotor.getEncoderRange()[1])
        return false;
    } else {
      System.out.println("!!! Could not find encoder range for aiming motor. Continuing...");
    }
    
    target = targetEncoder;
    return true;
  }

  public void execute() {
    double current = m_aimingMotor.getEncoderPosition();

    speed = MathUtil.clamp(profiledRotatePID.calculate(current, target), -1, 1);

    m_aimingMotor.set(speed);
  }

  public double getSpeed() {
    return speed;
  }

  public double getRotation(){
    return m_aimingMotor.getEncoderPosition() / Constants.kAimingDegreesPerRotation;
  }
}
