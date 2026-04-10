package frc.robot.subsystems;
import java.util.ArrayList;

// test
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation3d.*;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  private static ArrayList<Double> recentTimestamps = new ArrayList<>();
  QuestNav questNav;
  final Transform3d ROBOT_TO_QUEST = new Transform3d(.16, 0.0,0, new Rotation3d(0, 0, 0));
  final Transform3d QUEST_TO_ROBOT = new Transform3d(-.16, 0.0, -0, new Rotation3d(0, 0, 0));
  static Drive driveSubsystem;
  final Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.00000001, // Trust down to 1cm in X direction
          0.00000001, // Trust down to 1cm in Y direction
          .000002 // Trust down to 2 degrees rotational
          );

  public QuestNavSubsystem(Drive m_driveSubsystem) {
    driveSubsystem = m_driveSubsystem;
    questNav = new QuestNav();
    // Assume this is the requested reset pose

    // Pose3d robotPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    // // System.out.println("\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-\n-" +
    // // robotPose.getRotation());
    // resetPose(robotPose);
  }

  public void resetPose(Pose2d newPose) {
    
    Pose3d setPose = new Pose3d(newPose).transformBy(ROBOT_TO_QUEST);
    if (questNav == null) {
      System.out.println("quest not found (rs)");
      return;
    }
    questNav.setPose(setPose);
    questNav.setPose(setPose);
  }

  @Override
  public void periodic() {
    if (questNav == null) {
      System.out.println("quest not found (hs)");
      return;
    }
    questNav.commandPeriodic();

    // First, Declare our geometrical transform from the robot center to the Quest

    /* X: Positive -> Forward from robot center
    Y: Positive -> Left from robot center
    Rotation: Positive -> Counter Clockwise */

    // // Get the latest pose data frames from the Quest
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    if (poseFrames.length == 0) {
      SmartDashboard.putBoolean("QuestNav", false);
    } else {
      SmartDashboard.putBoolean("QuestNav", true);
      for (PoseFrame questFrame : poseFrames) {
        // Get the pose of the Quest
        Pose3d questPose = questFrame.questPose3d();
        // Get timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();
        

        // Transform by the mount pose to get your robot pose
        Pose3d robotPose = questPose.transformBy(QUEST_TO_ROBOT);

        Pose2d realRobotPose =
            new Pose2d(robotPose.getX(), robotPose.getY(), robotPose.getRotation().toRotation2d());

        // Add the measurement to our estimator
        driveSubsystem.addVisionMeasurement(realRobotPose, timestamp, QUESTNAV_STD_DEVS);
        double[] sentPoseData = new double[]{realRobotPose.getX(),realRobotPose.getY(),realRobotPose.getRotation().getDegrees()};
        SmartDashboard.putNumberArray("Quest pose data", sentPoseData);


      
        // SmartDashboard.putNumber("Quest rot", questPose.getRotation2d().getDegrees());
        // System.out.println("lock tf in");
      }
    }
  }

  public static double[] getVelocity() throws Exception{
    for(int i = recentTimestamps.size() - 3; i >= 0; i--){
      if (recentTimestamps.get(i) < recentTimestamps.get(recentTimestamps.size() - 1) - 100) continue;

      double t3 = RobotController.getTime() / Math.pow(10,6) - .2;
      double t2 = RobotController.getTime() / Math.pow(10,6) - .06;
      double t1 = RobotController.getTime() / Math.pow(10,6);
      Pose2d pose3, pose2, pose1;
      try{
        pose3 = driveSubsystem.samplePoseEstimate(t3);
        pose2 = driveSubsystem.samplePoseEstimate(t2);
        pose1 = driveSubsystem.samplePoseEstimate(t1);
      } catch(Exception e){
          throw new Exception("Pose Estimator buffer empty");
      }
      // SmartDashboard.putNumber("Mr. X", (pose3.getX()-pose1.getX())/(t3-t1));
      // SmartDashboard.putNumber("Mr. Y", (pose3.getY()-pose1.getY())/(t3-t1));
      return new double[]{(pose2.getX()-pose1.getX())/(t2-t1), (pose2.getY()-pose1.getY())/(t2-t1)};
    }

    throw new Exception("Timestamps out of date");
  }
}
