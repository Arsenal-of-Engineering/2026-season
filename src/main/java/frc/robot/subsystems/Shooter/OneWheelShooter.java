package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OneWheelShooter extends SubsystemBase {
  private final FlywheelIO _io;
  private final FlywheelIOInputsAutoLogged _inputs = new FlywheelIOInputsAutoLogged();
  private final double kP = 0.0004;
  private final double kI = 0;
  private final double kD = 0;
  private final double kV = 0.00014; // FeedFoward
  private final double maxAcc = 20000.0;
  private SparkMax shootMotor = new SparkMax(Constants.shooterMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController shootMotorPID = shootMotor.getClosedLoopController();

  public OneWheelShooter(FlywheelIO io) {
    _io = io;
  }

  @Override
  public void periodic() {
      _io.updateInputs(_inputs);
      Logger.processInputs("Shooter",_inputs);
  }
  
  public Command runSpeed(AngularVelocity speed) {
    return startEnd(
        () -> {shootMotorPID.setSetpoint(speed.in(RPM), ControlType.kMAXMotionVelocityControl);},
        () -> {shootMotorPID.setSetpoint(0.0, ControlType.kMAXMotionVelocityControl);});
  }
}
