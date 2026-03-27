package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.robotMOI;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.ImmutableAngularVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.ShuffleboardInfo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.OneWheelShooter;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.SimpleMoveMotor;
import frc.robot.subsystems.SolveTrajectoryPower;
import frc.robot.subsystems.drive.Drive;
import gg.questnav.questnav.QuestNav;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Transform2d;
/**
 * Uses the robots position and velocity to calculate and set a shooting angle for the turret, then start shooting when locked
 */
public class AimTarget extends Command {
  private AimTurret m_aimTurret;
  private Drive m_drive;
  private Timer timer = new Timer();
  private OneWheelShooter m_oneWheelShooter;
  private Supplier<Integer> calcRPMAdjSupplier;
  private boolean isRotationCorrection = false;
  private double turretTarget = 0;
  private PIDController rotatePID =
      new PIDController(
          .02, 0.0, 0.00, .02);
  final Transform2d ROBOT_TO_SHOOTER;
  private Side side;
  private Goal goal;

  public enum Side{
    LEFT,
    RIGHT
  }

  public enum Goal{
    HUB,
    CLOSEST_PASS
  }

  public AimTarget( AimTurret aimTurret, OneWheelShooter oneWheelShooter, Drive drive, Side side, Supplier<Integer> calcRPMAdjSupplier) {
    goal = Goal.HUB;
    this.calcRPMAdjSupplier = calcRPMAdjSupplier;
    this.side = side;
    switch(side){
      case LEFT:
        ROBOT_TO_SHOOTER = new Transform2d(.07, .02, new Rotation2d());
        break;
      case RIGHT:
      default:
        ROBOT_TO_SHOOTER = new Transform2d(.07, -.02, new Rotation2d());
        break;
    }
    
    rotatePID.setSetpoint(-45);
    m_aimTurret = aimTurret;
    m_oneWheelShooter = oneWheelShooter;
    m_drive = drive;
    addRequirements(m_oneWheelShooter);
    
    
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  public void execute() {

    Pose2d robotPose = m_drive.getPose();

    
    

    Pose2d turretPose = robotPose.transformBy(ROBOT_TO_SHOOTER);
    SmartDashboard.putNumber("TurrX", turretPose.getX());
    SmartDashboard.putNumber("TurrY", turretPose.getY());

    double xTarget, yTarget;
    switch(goal){
      case CLOSEST_PASS:
        xTarget = 1;
        yTarget = m_drive.getPose().getY() > 4 ? 8 - 1 : 1;
        break;
      case HUB:
      default:
        xTarget = 4.6;
        yTarget = 4;
    }
    System.out.println("Let's head to " + xTarget + ", " + yTarget);
    double[] robotGoal;
    ChassisSpeeds chassisSpeeds = m_drive.getChassisSpeeds();
    try{
      double[] speeds = QuestNavSubsystem.getVelocity();
      // balls are falling short on back up
      double xSpeed = speeds[0] < 0 ? speeds[0] * 1.5 : speeds[0];
      robotGoal = SolveTrajectoryPower.calculate(turretPose.getX(), -xSpeed, turretPose.getY(), -speeds[1], xTarget, yTarget, side == Side.LEFT ? 1 : -1, goal == Goal.HUB ? 2 : 0);
      SmartDashboard.putNumber("calc Vx", speeds[0]);
      SmartDashboard.putNumber("calc Vy", speeds[1]);
    } catch (Exception e){
      System.out.println(e.getMessage());
      // balls are falling short on back up
      double xSpeed = chassisSpeeds.vxMetersPerSecond < 0 ? chassisSpeeds.vxMetersPerSecond * 1.5 : chassisSpeeds.vxMetersPerSecond;
      robotGoal = SolveTrajectoryPower.calculate(turretPose.getX(), -chassisSpeeds.vxMetersPerSecond, turretPose.getY(), -chassisSpeeds.vyMetersPerSecond, xTarget, yTarget, side == Side.LEFT ? 1 : -1, goal == Goal.HUB ? 2 : 0);
    }
    SmartDashboard.putNumber("spark Vx", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("spark Vy", chassisSpeeds.vyMetersPerSecond);
    
    AngularVelocity shooterVelocity = RPM.of(robotGoal[1] + calcRPMAdjSupplier.get());
    //m_oneWheelShooter.setSpeed(RPM.of(rpmSelection.getSelected()));
    m_oneWheelShooter.setSpeed(shooterVelocity);
    
    SmartDashboard.putNumber("shooter vel", robotGoal[1] + calcRPMAdjSupplier.get());
  
    turretTarget = robotGoal[0] - robotPose.getRotation().getDegrees();
    SmartDashboard.putNumber("turret deg", turretTarget);
    
    isRotationCorrection = !m_aimTurret.setTarget(-turretTarget);
    m_aimTurret.setAngleTarget(robotGoal[2]);
    SmartDashboard.putNumber("Angle target " + (side == Side.LEFT ? "L" : "R"), robotGoal[2]);
    

    if (Math.abs(m_aimTurret.getRotation() + turretTarget) < 7.5){
        SmartDashboard.putBoolean("Ready on Aim", true);
        if (m_oneWheelShooter.isAtSetpoint() && timer.get() > 1){
        }
    } else {
        SmartDashboard.putBoolean("Ready on Aim", false);
    }
    // System.out.println("I'm getting " + robotGoal[0] + "deg at " + (robotGoal[1]+calcRPMAdjSupplier.get()) + "rpm with altitude: " + robotGoal[2]);
  }
  /**
   * Sets the command to target the specified goal location
   * @param newGoal The new goal to target
   */
  public void setGoal(Goal newGoal){
    goal = newGoal;
  }

  // public double getCorrectionSpeed(){
  //   if (!isRotationCorrection) return 0.0;
  //   return rotatePID.calculate(turretTarget);
  // }

  public void end(boolean interrupted){
    m_oneWheelShooter.setSpeed(RPM.of(0));
    timer.stop();
  }

  // public boolean isRotationCorrecting(){
  //   return isRotationCorrection;
  // }

}
