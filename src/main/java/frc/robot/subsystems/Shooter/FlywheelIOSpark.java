package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelIOSpark  implements FlywheelIO {
    private final double kMaxAccelleration=20000;

    private final SparkBase _motor ;
    private final SparkBaseConfig _config;
    private final RelativeEncoder _encoder;
    private final SparkClosedLoopController _controller;
    
    
    public FlywheelIOSpark(int canId, boolean isSparkFlex) {
        // create the requested type of Spark 
        _motor = isSparkFlex
                ? new SparkFlex(canId, MotorType.kBrushless)
                : new SparkMax(canId, MotorType.kBrushless);
        // create a corresponding config and fill in common settings
        _config = isSparkFlex ? new SparkFlexConfig() : new SparkMaxConfig();
        _config.idleMode(IdleMode.kCoast);
        _config.closedLoop
                .outputRange(0, 1)
                .maxMotion.maxAcceleration(kMaxAccelleration);
        // send the settings to the Spark
        _motor.configure(_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _encoder = _motor.getEncoder();
        _controller = _motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.data = new FlywheelIOData(
            _controller.getMAXMotionSetpointVelocity(),
            _encoder.getVelocity(), 
            _motor.getAppliedOutput()
            );       
    }

    @Override
    public void setPid(double kP, double kI, double kD){
        _config.closedLoop.pid(kP,kI,kD);
        _motor.configure(_config, ResetMode.kNoResetSafeParameters, 
            PersistMode.kNoPersistParameters);
    }

    @Override
    public void setSpeed(AngularVelocity speed){
        _controller.setSetpoint(
            speed.in(RPM),
            ControlType.kMAXMotionVelocityControl);
    }

}
