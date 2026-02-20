// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.MoveArc;
import frc.robot.commands.AimTarget;
import frc.robot.commands.AimTurret;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.SimpleMoveMotor;
import frc.robot.subsystems.OneWheelShooter;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // private final ShootMotors m_shootMotors = new ShootMotors();
  private static QuestNavSubsystem m_questNav;
  private MoveArc m_leftArc, m_rightArc;
  private AimTurret m_rotate;
  
  private final OneWheelShooter m_oneShootMotor = new OneWheelShooter();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private SimpleMoveMotor m_intakeMotor = new SimpleMoveMotor(Constants.kIntakeMotorCanId);
  private SimpleMoveMotor m_pacerMotor = new SimpleMoveMotor(Constants.kPacerMotorCanId);
  private SimpleMoveMotor m_aimingMotor = new SimpleMoveMotor(Constants.kAimingMotorCanId);
  private SimpleMoveMotor m_conveyorMotor = new SimpleMoveMotor(Constants.kConveyorMotorCanId);
  private SimpleMoveMotor m_feederMotor = new SimpleMoveMotor(Constants.kFeederMotorCanId);
  private SimpleMoveMotor m_angleMotor = new SimpleMoveMotor(Constants.kAngleMotorCanId);
  private AimTurret m_aimTurret = new AimTurret(m_aimingMotor);
  private Joystick m_joystick = new Joystick(Constants.kJoystickId);
  private AimTarget m_aimTarget;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    ConfigureNamedCommands();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    m_questNav = new QuestNavSubsystem(drive);

    // Set up auto routines

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_aimTarget = new AimTarget(m_aimTurret, m_oneShootMotor, drive);

    // Configure the button bindings
    configureButtonBindings();
  }

  public double getYSpeed() {
    // if (m_leftArc.isScheduled()) {
    //   return m_leftArc.getYSpeed();
    // }
    // if (m_rightArc.isScheduled()) {
    //   return m_rightArc.getYSpeed();
    // }

    return -MathUtil.applyDeadband(controller.getLeftY(), .08);
  }

  public double getXSpeed() {
    // if (m_leftArc.isScheduled()) {
    //   return m_leftArc.getXSpeed();
    // }
    // if (m_rightArc.isScheduled()) {
    //   return m_rightArc.getXSpeed();
    // }

    return -MathUtil.applyDeadband(controller.getLeftX(), .08);
  }

  public double getRotSpeed() {
    // if (m_rotate.isScheduled()) {
    //   return m_rotate.getSpeed();
    // }

    return -controller.getRightX();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> getYSpeed(), () -> getXSpeed(), () -> getRotSpeed()));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller.x().onTrue(Commands.runOnce(() -> m_shootMotors.shoot(-1)));
    // controller.x().onFalse(Commands.runOnce(() -> m_shootMotors.shoot(0)));
    controller.leftBumper().whileTrue(new RepeatCommand(new InstantCommand(() -> m_aimingMotor.set(-.25))));
    controller.rightBumper().whileTrue(new RepeatCommand(new InstantCommand(() -> m_aimingMotor.set(.25))));
    controller.leftBumper().onFalse(new RepeatCommand(new InstantCommand(() -> m_aimingMotor.set(0))));
    controller.rightBumper().onFalse(new RepeatCommand(new InstantCommand(() -> m_aimingMotor.set(0))));

    controller.y().whileTrue(m_oneShootMotor.runSpeed(RPM.of(3000.0)));

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));

    controller.a().onTrue(new InstantCommand(() -> m_aimTurret.setTarget(m_aimTurret.getRotation())));
    controller.a().whileTrue(m_aimTarget.alongWith(m_aimTurret));

    // configure joystick buttons (<1st arg>) to do <2nd arg> on press and <3rd arg> on release.
    joystickInstantToggle(1, () -> m_intakeMotor.set(1), () -> m_intakeMotor.set(0));
    joystickInstantToggle(2, () -> {m_feederMotor.set(-1); m_conveyorMotor.set(1);}, () -> {m_feederMotor.set(0); m_conveyorMotor.set(0);});
    joystickInstantToggle(5, () -> {m_feederMotor.set(1); m_conveyorMotor.set(-1);}, () -> {m_feederMotor.set(0); m_conveyorMotor.set(0);});
    joystickInstantToggle(6, () -> m_intakeMotor.set(-1), () -> m_intakeMotor.set(0));
    joystickInstantToggle(8, () -> m_pacerMotor.set(1), () -> m_pacerMotor.set(0));
    joystickInstantToggle(10, () -> m_pacerMotor.set(-1), () -> m_pacerMotor.set(0));
    joystickInstantToggle(7, () -> m_angleMotor.set(.15), () -> m_angleMotor.set(0));
    joystickInstantToggle(9, () -> m_angleMotor.set(-.15), () -> m_angleMotor.set(0));
    joystickInstantToggle(11, () -> m_oneShootMotor.setSpeed(RPM.of(4000)), () -> m_oneShootMotor.setSpeed(RPM.of(0)));
  }

  /**
   * Runs an instant command when the joystick button is pressed and released
   * @param button The button that must be pressed
   * @param onTrue The action to run when the button is just pressed
   * @param onFalse The action to run when the button is just released
   */
  private void joystickInstantToggle(int button, Runnable onTrue, Runnable onFalse){
    new JoystickButton(m_joystick, button)
      .onTrue(new InstantCommand(onTrue));
    new JoystickButton(m_joystick, button)
      .onFalse(new InstantCommand(onFalse));
  }

  private void ConfigureNamedCommands() {
    NamedCommands.registerCommand(
        "Shoot", new InstantCommand(() -> System.out.println("Im shooting it")));
    NamedCommands.registerCommand(
        "Intake", new InstantCommand(() -> System.out.println("im intaking fuel")));
    NamedCommands.registerCommand(
        "Climb", new InstantCommand(() -> System.out.println("surely there is a climb design")));

    NamedCommands.registerCommand("Shoot", m_aimTurret);
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> {m_conveyorMotor.set(1.0); m_feederMotor.set(-1);m_intakeMotor.set(1.0);}));
    NamedCommands.registerCommand("Process", new InstantCommand(() -> {m_conveyorMotor.set(1.0);m_feederMotor.set(-1);}));
  }

  public static void resetQuestPose(Pose2d pose){

    m_questNav.resetPose(new Pose3d(pose));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var alliance = DriverStation.getAlliance();
    return autoChooser.get().handleInterrupt(
        () -> {
                DriveCommands.joystickDrive(drive, 0, 0, 0);
                
        });
    }
}
