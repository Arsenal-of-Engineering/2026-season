package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SimpleMoveMotor extends SubsystemBase {
    private SparkMax theMotor;
    private SparkClosedLoopController pidController;
    private double[] encoderRange = null;
    private double speed = 0;
    public SimpleMoveMotor(int id){theMotor = new SparkMax(id, MotorType.kBrushless);}

    public SimpleMoveMotor(int id, double encoderMin, double encoderMax){
        theMotor = new SparkMax(id, MotorType.kBrushless);
        encoderRange = new double[]{encoderMin,encoderMax};
    }

    public void initPID(double kP, double kI, double kD){
        pidController = theMotor.getClosedLoopController();
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(kP, kI, kD)
            .outputRange(-1, 1);
        config.idleMode(IdleMode.kBrake);
        if (theMotor.getDeviceId() == Constants.kLeftAngleMotorCanId || theMotor.getDeviceId() == Constants.kRightAngleMotorCanId){
            config.softLimit.forwardSoftLimit(0);
            config.softLimit.forwardSoftLimitEnabled(true);
            config.softLimit.reverseSoftLimit(-20);
            config.softLimit.reverseSoftLimitEnabled(true);
        }

        theMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController.setSetpoint(theMotor.getEncoder().getPosition(), ControlType.kPosition);
    }

    public void setSetpoint(double setpoint){
        if (pidController == null) return;

        pidController.setSetpoint(setpoint, ControlType.kPosition);
        System.out.println("run my pitch to " + setpoint);
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
        if(theMotor.getDeviceId() == Constants.kLeftAimingMotorCanId){
            SmartDashboard.putNumber("Aiming encoder L", getEncoderPosition());
        }
        if(theMotor.getDeviceId() == Constants.kLeftAngleMotorCanId){
            SmartDashboard.putNumber("Angle encoder L", getEncoderPosition());
        }
        if(theMotor.getDeviceId() == Constants.kRightAngleMotorCanId){
            SmartDashboard.putNumber("Angle encoder R", getEncoderPosition());
        }
        if(theMotor.getDeviceId() == Constants.kRightAimingMotorCanId){
            SmartDashboard.putNumber("Aiming encoder R", getEncoderPosition());
        }
    }

    public double getEncoderPosition(){
        return theMotor.getEncoder().getPosition();
    }

    public double[] getEncoderRange(){
        return encoderRange;
    }
}