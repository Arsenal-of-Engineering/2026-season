package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private SparkFlex theMotor;
    private SparkFlex theOtherMotor;
    private double speed = 0;
    public Intake(int id, int id2){
        theMotor = new SparkFlex(id, MotorType.kBrushless);
        theOtherMotor = new SparkFlex(id2, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(80000);

        theMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        theOtherMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void set(double newSpeed) {
        //explode
        speed = newSpeed;
    }


    @Override
    public void periodic() {
        double rpm = Math.abs(theMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber(theMotor.getDeviceId() + " rpming", rpm);
        double output = rpm < 450000 ? 1 : 0;
        theMotor.set(output * speed);
        theOtherMotor.set(output*speed);
    }

    public double getEncoderPosition(){
        return theMotor.getEncoder().getPosition();
    }

}