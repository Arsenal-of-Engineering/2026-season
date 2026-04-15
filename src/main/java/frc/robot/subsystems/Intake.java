package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private SparkFlex theMotor;
    private SparkFlex theOtherMotor;
    private double speed = 0;
    private SendableChooser<Integer> intakeRpmSelector = new SendableChooser<>();
    private SparkClosedLoopController motorPID, otherMotorPID;
    public Intake(int id, int id2){
        theMotor = new SparkFlex(id, MotorType.kBrushless);
        theOtherMotor = new SparkFlex(id2, MotorType.kBrushless);

        motorPID = theMotor.getClosedLoopController();
        otherMotorPID = theOtherMotor.getClosedLoopController();
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(0.0004, 0, 0)
            .outputRange(-1,1);
        config.idleMode(IdleMode.kCoast);
        config.closedLoop.maxMotion.maxAcceleration(25000);

        theMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        theOtherMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    


    public void set(double newSpeed) {
        //explode
        speed = newSpeed;
        motorPID.setSetpoint(4000 * newSpeed, ControlType.kMAXMotionVelocityControl);
        otherMotorPID.setSetpoint(4000 * newSpeed, ControlType.kMAXMotionVelocityControl);
    }


    @Override
    public void periodic() {
    }

    public double getEncoderPosition(){
        return theMotor.getEncoder().getPosition();
    }

}