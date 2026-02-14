package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MoveMotors extends SubsystemBase {
    private SparkMax TheMotor;
    public MoveMotors(int id){TheMotor = new SparkMax(id, MotorType.kBrushless);}
    public void move(double speed) {
        //explode
        TheMotor.set(speed);
    }
}