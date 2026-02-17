package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleMoveMotor extends SubsystemBase {
    private SparkMax theMotor;
    private double[] encoderRage = null;
    public SimpleMoveMotor(int id){theMotor = new SparkMax(id, MotorType.kBrushless);}

    public SimpleMoveMotor(int id, double encoderMin, double encoderMax){
        theMotor = new SparkMax(id, MotorType.kBrushless);
        encoderRage = new double[]{encoderMin,encoderMax};
    }

    public void set(double speed) {
        // don't move past encoder
        if (encoderRage != null){
            if (speed < 0 && theMotor.getAbsoluteEncoder().getPosition() < encoderRage[0]){
                speed = 0;
            }
            if (speed > 0 && theMotor.getAbsoluteEncoder().getPosition() > encoderRage[1]){
                speed = 0;
            }
        }
        //explode
        theMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder " + theMotor.getDeviceId(), getEncoderPosition());
    }

    public double getEncoderPosition(){
        return theMotor.getAbsoluteEncoder().getPosition();
    }

    public double[] getEncoderRange(){
        return encoderRage;
    }
}