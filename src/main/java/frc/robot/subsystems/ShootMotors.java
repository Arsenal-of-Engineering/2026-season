package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShootMotors extends SubsystemBase {
  private SparkMax shootMotor1 = new SparkMax(10, MotorType.kBrushless);
  private SparkMax shootMotor2 = new SparkMax(11, MotorType.kBrushless);
  private SparkMax armMotor = new SparkMax(12, MotorType.kBrushless);

  public void shoot(double speed) {
    shootMotor1.set(speed);
    shootMotor2.set(-speed);
  }

  public void arm(double speed) {
    armMotor.set(speed);
  }
}
