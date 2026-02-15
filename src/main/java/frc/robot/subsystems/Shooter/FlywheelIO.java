package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.AngularVelocity;


public interface FlywheelIO{

    @AutoLog
    class FlywheelIOInputs{
        public FlywheelIOData data =new FlywheelIOData(0, 0, 0);
    }

    record FlywheelIOData(
        double velocitySetpoint,
        double velocityRPM,
        double appliedVoltage
    ){}

    default void updateInputs(FlywheelIOInputs inputs){  }

    default void setPid(double kP, double kI, double kD){};
    default void setSpeed(AngularVelocity speed){}
}