package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


public class OneWheelShooter extends SubsystemBase {
  // private final double kP = 0.00125;
  // private final double kI = 0;
  // private final double kD = 0;
  // private final double kV = 0.0; // FeedFoward
  // private final double kS = .01;
  // private final double kP = 0.001;
  private  double kP = 0.4;
  private final double kI = 0;
  private final double kD = 0/* .00036*/;
  private  double kV = 0.00264; // FeedFoward
  private final double kS = 0;
  private final double kA = .00;
  private final double maxAcc = 2000000.0;
  private SparkFlex shootMotor;
  private boolean isLeft = false;
  private SparkClosedLoopController shootMotorPID;
  private SimpleMotorFeedforward feedforward;
  private double setpoint = 0;

  public OneWheelShooter(int id, boolean issLeft) {

    this.isLeft = issLeft;
    if (!isLeft){
      //kV *= 32 / 35.0;
      //kP *= 32 / 35.0;
    }
    shootMotor = new SparkFlex(id, MotorType.kBrushless);
    shootMotorPID = shootMotor.getClosedLoopController();
    MAXMotionConfig motionConfig = new MAXMotionConfig();
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.outputRange(isLeft ? -1 : 0, isLeft ? 0 : 1)
        .pid(kP, kI, kD).feedForward.kV(kS);
    config.voltageCompensation(12);
        
        
    config.idleMode(IdleMode.kCoast);
    config.closedLoop.maxMotion.maxAcceleration(maxAcc);

    shootMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // public Command runSpeed(AngularVelocity speed) {
  //   return startEnd(
  //       () -> {shootMotorPID.setSetpoint((isLeft ? -1 : 1) * speed.in(RPM), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);},
  //       () -> {shootMotorPID.setSetpoint(0, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);}
  //   );
  // }
  @Override
  public void periodic() {
      
      double volts = feedforward.calculateWithVelocities(shootMotor.getEncoder().getVelocity(), setpoint);
      if (setpoint == 0)
        volts = 0;
      // System.out.println("BzZZzZ: " + volts);
      
      shootMotorPID.setSetpoint(setpoint, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, volts);
      if (setpoint == 0){
        shootMotor.set(0);
        return;
      }
      
      double vel = shootMotor.getEncoder().getVelocity();
      double del = (vel - setpoint) * (isLeft ? -1 : 1);
      double pwr = 0;
      double dir = (setpoint - vel) / del;
      // if (del > 1000){
      //   pwr = 1;
      // } else if (del > 500){
      //   pwr = .75;
      // }else if (del > 200){
      //   pwr = .6;
      // }else if (del > 100){
      //   pwr = .5;
      // }

      // if (del < -200){
      //   shootMotor.set(isLeft ? -1 : 1);
      // } else {
      //   double volts = feedforward.calculateWithVelocities(shootMotor.getEncoder().getVelocity(), setpoint);
      //   shootMotor.setVoltage(volts);
      // }

      System.out.println((isLeft ? "L" : "R") + ": " + shootMotor.getEncoder().getVelocity());
  }
  public void setSpeed(AngularVelocity speed) {
    setpoint = (isLeft ? -1 : 1) * speed.in(RPM);
    // shootMotorPID.setSetpoint((isLeft ? -1 : 1) * speed.in(RPM), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    // System.out.println((isLeft ? -1 : 1) * speed.in(RPM));
    // System.out.println("lets run it to: " + shootMotorPID.getSetpoint() + " or is it: " + shootMotorPID.getMAXMotionSetpointVelocity());
  }

  public boolean isAtSetpoint(){
    return shootMotorPID.isAtSetpoint() && shootMotorPID.getMAXMotionSetpointVelocity() != 0;
  }
}
