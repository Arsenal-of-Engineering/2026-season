package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleMoveMotor extends SubsystemBase {
    private SparkMax TheMotor;
    private double[] encoderRage = null;
    public SimpleMoveMotor(int id){TheMotor = new SparkMax(id, MotorType.kBrushless);}

    public SimpleMoveMotor(int id, double encoderMin, double encoderMax){
        TheMotor = new SparkMax(id, MotorType.kBrushless);
        encoderRage = new double[]{encoderMin,encoderMax};
    }

    public void set(double speed) {
        if (encoderRage != null){
            if (speed < 0 && TheMotor.getAbsoluteEncoder().getPosition() < encoderRage[0]){
                speed = 0;
            }
            if (speed > 0 && TheMotor.getAbsoluteEncoder().getPosition() > encoderRage[1]){
                speed = 0;
            }
        }
        //explode
        TheMotor.set(speed);
    }
}