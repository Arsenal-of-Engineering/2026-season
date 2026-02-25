package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SimpleMoveMotor extends SubsystemBase {
    private SparkMax theMotor;
    private double[] encoderRange = null;
    private double speed = 0;
    public SimpleMoveMotor(int id){theMotor = new SparkMax(id, MotorType.kBrushless);}

    public SimpleMoveMotor(int id, double encoderMin, double encoderMax){
        theMotor = new SparkMax(id, MotorType.kBrushless);
        encoderRange = new double[]{encoderMin,encoderMax};
    }

    public void set(double newSpeed) {
        // don't move past encoder
        if (encoderRange != null){
            if (newSpeed < 0 && theMotor.getEncoder().getPosition() < encoderRange[0]){
                newSpeed = 0;
            }
            if (newSpeed > 0 && theMotor.getEncoder().getPosition() > encoderRange[1]){
                newSpeed = 0;
            }
        }
        // System.out.println("and I'm gooning it " + newSpeed + " times");
        //explode
        theMotor.set(newSpeed);
        speed = newSpeed;
    }

    @Override
    public void periodic() {
        // if (theMotor.getDeviceId() == Constants.kAimingMotorCanId){
        //     SmartDashboard.putNumber("encoder " + theMotor.getDeviceId(), getEncoderPosition() * Constants.kAimingDegreesPerRotation);
        // } else {
        //     SmartDashboard.putNumber("encoder " + theMotor.getDeviceId(), getEncoderPosition());   
        // }
        if (encoderRange != null){
            if (speed < 0 && theMotor.getEncoder().getPosition() < encoderRange[0]){
                speed = 0;
                theMotor.set(0);
            }
            if (speed > 0 && theMotor.getEncoder().getPosition() > encoderRange[1]){
                speed = 0;
                theMotor.set(0);
            }
        }
    }

    public double getEncoderPosition(){
        return theMotor.getEncoder().getPosition();
    }

    public double[] getEncoderRange(){
        return encoderRange;
    }
}