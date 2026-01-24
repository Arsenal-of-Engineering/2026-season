package frc.robot.subsystems.drive;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
    QuestNav questNav = new QuestNav();
    final Transform3d ROBOT_TO_QUEST = new Transform3d( );
    Drive driveSubsystem;
    final Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.00002, // Trust down to 2cm in X direction
            0.00002, // Trust down to 2cm in Y direction
            0.000035 // Trust down to 2 degrees rotational
        );



    public QuestNavSubsystem(/*maybe add robot pose info here */Drive driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        // Assume this is the requested reset pose
        Pose3d robotPose = new Pose3d( 0,0,0,new Rotation3d(0,0,0) );

        resetPose(robotPose);
    } 

    public void resetPose(Pose3d newPose){
        newPose = newPose.transformBy(ROBOT_TO_QUEST);
        questNav.setPose(newPose);
    }
    
    @Override
    public void periodic() {
        questNav.commandPeriodic();


        // First, Declare our geometrical transform from the robot center to the Quest
        
        /* X: Positive -> Forward from robot center
            Y: Positive -> Left from robot center
            Rotation: Positive -> Counter Clockwise */

        // // Get the latest pose data frames from the Quest
        // PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

        // if (poseFrames.length > 0) {
        //     // Get the most recent Quest pose
        //     Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();

        //     // Transform by the mount pose to get your robot pose
        //     Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
        // }


        if (questNav.isTracking()) {
            // Get the latest pose data frames from the Quest
            PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
        
            // Loop over the pose data frames and send them to the pose estimator
            for (PoseFrame questFrame : questFrames) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();
        
                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
                
                // Add the measurement to our estimator
                driveSubsystem.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
            }
        }
    }
}
