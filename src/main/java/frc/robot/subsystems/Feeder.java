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

public class Feeder extends SubsystemBase {
    private SparkMax topMotor;
    private SparkMax bottomMotor;
    public Feeder(int topId, int bottomId){
        topMotor = new SparkMax(topId, MotorType.kBrushless);
        bottomMotor = new SparkMax(bottomId, MotorType.kBrushless);
    }

    public void set(double newSpeed) {
        topMotor.set(newSpeed);
        bottomMotor.set(-newSpeed);
    }

}