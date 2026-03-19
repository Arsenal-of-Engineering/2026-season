package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OneWheelShooter extends SubsystemBase {
  private final double kP = 0.0006;
  private final double kI = 0;
  private final double kD = 0;
  private final double kV = 0.00014; // FeedFoward
  private final double maxAcc = 20000.0;
  private SparkFlex shootMotor;
  private SparkClosedLoopController shootMotorPID;

  public OneWheelShooter(int id) {
    shootMotor = new SparkFlex(id, MotorType.kBrushless);
    shootMotorPID = shootMotor.getClosedLoopController();
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD)
        .outputRange(0, 1)
        .feedForward
        .kV(kV);
    config.idleMode(IdleMode.kCoast);
    config.closedLoop.maxMotion.maxAcceleration(maxAcc);

    shootMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command runSpeed(AngularVelocity speed) {
    return startEnd(
        () -> {shootMotorPID.setSetpoint(speed.in(RPM), ControlType.kMAXMotionVelocityControl);},
        () -> {shootMotorPID.setSetpoint(0, ControlType.kMAXMotionVelocityControl);}
    );
  }

  public void setSpeed(AngularVelocity speed) {
    shootMotorPID.setSetpoint(speed.in(RPM), ControlType.kMAXMotionVelocityControl);
  }

  public boolean isAtSetpoint(){
    return shootMotorPID.isAtSetpoint() && shootMotorPID.getSetpoint() != 0;
  }
}
