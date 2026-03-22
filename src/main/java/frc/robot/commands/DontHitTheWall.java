package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AddressableLED;
// import frc.robot.ShuffleboardInfo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SimpleMoveMotor;
import frc.robot.subsystems.drive.Drive;

public class DontHitTheWall extends Command {

    private Supplier<Pose2d> poseSupplier;

  public DontHitTheWall(Supplier<Pose2d> robotPoseSupplier) {
    poseSupplier = robotPoseSupplier;
  }

  @Override
  public boolean isFinished() {
      return poseSupplier.get().getX() < 2;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
        System.out.println("AHHH THAT WALL IS CLOSE!!!1!");
  }
}
