package frc.robot.subsystems;
// test
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation3d.*;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  QuestNav questNav;
  final Transform3d ROBOT_TO_QUEST = new Transform3d(9 * .0254, -9 * .0254, 0, new Rotation3d(0, 0, 0));
  final Transform3d QUEST_TO_ROBOT = new Transform3d(-9 * .0254, 9 * .0254, -0, new Rotation3d(0, 0, 0));
  Drive driveSubsystem;
  final Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.01, // Trust down to 1cm in X direction
          0.01, // Trust down to 1cm in Y direction
          .2 // Trust down to 2 degrees rotational
          );

  public QuestNavSubsystem(/*maybe add robot pose info here */ Drive driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
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

    if (poseFrames.length > 0) {
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

        SmartDashboard.putNumber("Quest X", questPose.getX());
        SmartDashboard.putNumber("Quest Y", questPose.getY());
        // SmartDashboard.putNumber("Quest rot", questPose.getRotation2d().getDegrees());
        // System.out.println("lock tf in");
      }

      SmartDashboard.putBoolean("QuestNav", true);
    } else {
      SmartDashboard.putBoolean("QuestNav", false);
    }
  }
}
