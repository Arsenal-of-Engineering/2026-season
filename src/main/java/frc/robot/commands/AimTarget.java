package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.robotMOI;

import edu.wpi.first.epilogue.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.ImmutableAngularVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.ShuffleboardInfo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.OneWheelShooter;
import frc.robot.subsystems.SimpleMoveMotor;
import frc.robot.subsystems.SolveTrajectoryPower;
import frc.robot.subsystems.drive.Drive;
/**
 * Uses the robots position and velocity to calculate and set a shooting angle for the turret, then start shooting when locked
 */
public class AimTarget extends Command {
  private AimTurret m_aimTurret;
  private Drive m_drive;
  private OneWheelShooter m_oneWheelShooter;
  private SendableChooser<Integer> rpmSelection = new SendableChooser<>();

  public AimTarget(AimTurret aimTurret, OneWheelShooter oneWheelShooter, Drive drive) {
    m_aimTurret = aimTurret;
    m_oneWheelShooter = oneWheelShooter;
    m_drive = drive;
    addRequirements(m_oneWheelShooter);
    for(int i = 4000; i <= 20000; i+= 1000){
      rpmSelection.addOption("" + i, i);
    }
    rpmSelection.setDefaultOption("" + 4000, 4000);
    rpmSelection.close();
    SmartDashboard.putData("rpm chooser", rpmSelection);
  }

  public void execute() {

    Pose2d robotPose = m_drive.getPose();
    
    double[] robotGoal = SolveTrajectoryPower.calculate(robotPose.getX(), robotPose.getY(), 0, 0);

    AngularVelocity shooterVelocity = RPM.of(robotGoal[1]);
    m_oneWheelShooter.setSpeed(RPM.of(rpmSelection.getSelected()));
    //m_oneWheelShooter.setSpeed(shooterVelocity);
    
    SmartDashboard.putNumber("shooter vel", robotGoal[1]);
  
    double turretTarget = robotGoal[0] - robotPose.getRotation().getDegrees();
    SmartDashboard.putNumber("turret deg", turretTarget);
    SmartDashboard.putBoolean("Turret Check", m_aimTurret.setTarget(-turretTarget));

    if (Math.abs(m_aimTurret.getRotation() - robotGoal[0]) < 7.5){
        SmartDashboard.putBoolean("Ready on Aim", true);
    } else {
        SmartDashboard.putBoolean("Ready on Aim", false);
    }
    
  }

  public void end(boolean interrupted){
    m_oneWheelShooter.setSpeed(RPM.of(0));
  }
}
