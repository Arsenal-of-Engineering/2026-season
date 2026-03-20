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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.AimTarget.Goal;
import frc.robot.commands.AimTarget.Side;
import frc.robot.commands.AimTarget;
import frc.robot.commands.AimTurret;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  
  private final OneWheelShooter m_leftOneShootMotor = new OneWheelShooter(Constants.kLeftShooterMotorCanId);
  private final OneWheelShooter m_rightOneShootMotor = new OneWheelShooter(Constants.kRightShooterMotorCanId);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private SendableChooser<Pose2d> autoStartChooser;
 
  private SimpleMoveMotor m_intakeMotor = new SimpleMoveMotor(Constants.kIntakeMotorCanId);
  
  private SimpleMoveMotor m_leftAimingMotor = new SimpleMoveMotor(Constants.kLeftAimingMotorCanId);
  private SimpleMoveMotor m_rightAimingMotor = new SimpleMoveMotor(Constants.kRightAimingMotorCanId);
  private SimpleMoveMotor m_leftAngleMotor = new SimpleMoveMotor(Constants.kLeftAngleMotorCanId);
  private SimpleMoveMotor m_rightAngleMotor = new SimpleMoveMotor(Constants.kRightAngleMotorCanId);
  private SimpleMoveMotor m_conveyorMotor = new SimpleMoveMotor(Constants.kConveyorMotorCanId);
  private SimpleMoveMotor m_feederMotor = new SimpleMoveMotor(Constants.kFeederMotorCanId);
  
  private AimTurret m_leftAimTurret = new AimTurret(m_leftAimingMotor, m_leftAngleMotor, true);
  private AimTurret m_rightAimTurret = new AimTurret(m_rightAimingMotor, m_rightAngleMotor, false);
  private AimTurret m_autoLeftAimTurret = new AimTurret(m_leftAimingMotor, m_leftAngleMotor, true);
  private AimTurret m_autoRightAimTurret = new AimTurret(m_rightAimingMotor, m_rightAngleMotor, false);
  private Joystick m_joystick = new Joystick(Constants.kJoystickId);
  private AimTarget m_leftAimTarget, m_rightAimTarget, m_autoLeftAimTarget, m_autoRightAimTarget;

  // six seven
  private Servo leftServo = new Servo(6);
  private Servo rightServo = new Servo(7);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    
    

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
    m_leftAimTarget = new AimTarget(m_leftAimTurret, m_leftOneShootMotor, drive, Side.LEFT);
    m_rightAimTarget = new AimTarget( m_rightAimTurret, m_rightOneShootMotor, drive, Side.RIGHT);
    m_autoLeftAimTarget = new AimTarget( m_leftAimTurret, m_leftOneShootMotor, drive, Side.LEFT);
    m_autoRightAimTarget = new AimTarget( m_rightAimTurret, m_rightOneShootMotor, drive, Side.RIGHT);

    // Set up auto routines

    ConfigureNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoStartChooser = new SendableChooser<>();
    autoStartChooser.addOption("_Left...", new Pose2d(3.69, 6.435, new Rotation2d(0,0)));
    autoStartChooser.addOption("_Left Humble...", new Pose2d(3.69, 4.968, new Rotation2d(Math.toRadians(180))));
    autoStartChooser.addOption("_Center...", new Pose2d(3.703, 4.016, new Rotation2d(0,0)));
    autoStartChooser.addOption("_Right...", new Pose2d(3.716, 1.648, new Rotation2d(0,0)));
    autoStartChooser.setDefaultOption("_Center...", new Pose2d(3.703, 4.016, new Rotation2d(0,0)));
    autoStartChooser.close();
    SmartDashboard.putData("Auto Start", autoStartChooser);

    autoStartChooser.onChange(this::pushAutoPose);

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

    

    // Configure the button bindings
    configureButtonBindings();

    // Put the USB camera into network tables to show in SmartDashboard
    var camera0 = CameraServer.startAutomaticCapture();
    CameraServer.addCamera(camera0);
  }

  public void pushAutoPose(Pose2d pose){
    drive.setPose(pose);
  }

  public void extendHopper(){
    leftServo.set(.5);
    rightServo.set(.5);
  }

  public double getYSpeed() {
    // if (m_leftArc.isScheduled()) {
    //   return m_leftArc.getYSpeed();
    // }
    // if (m_rightArc.isScheduled()) {
    //   return m_rightArc.getYSpeed();
    // }

    return -MathUtil.applyDeadband(controller.getLeftY(), .1);
  }

  public double getXSpeed() {
    // if (m_leftArc.isScheduled()) {
    //   return m_leftArc.getXSpeed();
    // }
    // if (m_rightArc.isScheduled()) {
    //   return m_rightArc.getXSpeed();
    // }

    return -MathUtil.applyDeadband(controller.getLeftX(), .1);
  }

  public double getRotSpeed() {
    // if (m_aimTarget.isScheduled() && m_aimTarget.isRotationCorrecting()) {
    //   return m_aimTarget.getCorrectionSpeed();
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
    // controller.x().onFalse(Commands.runOnce(() -> m_shootMotors.shoot(0)));\


    controller.leftBumper().whileTrue(new RepeatCommand(new InstantCommand(() -> {m_leftAimingMotor.set(-.4);/*m_rightAimingMotor.set(-.25);*/})));
    controller.rightBumper().whileTrue(new RepeatCommand(new InstantCommand(() -> {m_leftAimingMotor.set(.4);/*m_rightAimingMotor.set(.25);*/})));
    controller.leftBumper().onFalse(new RepeatCommand(new InstantCommand(() -> {m_leftAimingMotor.set(0);/*m_rightAimingMotor.set(0);*/})));
    controller.rightBumper().onFalse(new RepeatCommand(new InstantCommand(() -> {m_leftAimingMotor.set(0);/*m_rightAimingMotor.set(0);*/})));
    // controller.leftBumper().onTrue(new InstantCommand(() -> {m_rightAimTurret.setTarget(-70);}));
    // controller.rightBumper().onTrue(new InstantCommand(() -> {m_rightAimTurret.setTarget(10);}));
    // controller.a().whileTrue(m_rightAimTurret);

    controller.leftTrigger(.5).onTrue(new InstantCommand(() -> drive.setPose(new Pose2d(3.677,4,new Rotation2d(0,0)))));

    controller.a().onTrue(new InstantCommand(() -> extendHopper()));
  
    controller.b().onTrue(new InstantCommand(() -> {m_rightAimTarget.setGoal(Goal.CLOSEST_PASS);m_leftAimTarget.setGoal(Goal.CLOSEST_PASS);}));
    controller.x().onTrue(new InstantCommand(() -> {m_rightAimTarget.setGoal(Goal.HUB);m_rightAimTarget.setGoal(Goal.HUB);}));

    controller.rightTrigger(.5).onTrue(new InstantCommand(() -> {m_rightAimTurret.setTarget(m_rightAimTurret.getRotation());m_leftAimTurret.setTarget(m_leftAimTurret.getRotation());}));
    controller.rightTrigger(.5).whileTrue(m_leftAimTarget.alongWith(m_leftAimTurret).alongWith(m_rightAimTarget).alongWith(m_rightAimTurret));

    

    // configure joystick buttons (<1st arg>) to do <2nd arg> on press and <3rd arg> on release.
    joystickInstantToggle(1, () -> m_intakeMotor.set(1), () -> m_intakeMotor.set(0));
    joystickInstantToggle(2, () -> m_feederMotor.set(-1), () -> m_feederMotor.set(0));
    // joystickInstantToggle(11, () -> m_conveyorMotor.set(1), () -> m_conveyorMotor.set(0));
    // joystickInstantToggle(9, () -> m_conveyorMotor.set(-1), () -> m_conveyorMotor.set(0));
    joystickInstantToggle(5, () -> m_feederMotor.set(1), () -> m_feederMotor.set(0));
    joystickInstantToggle(6, () -> m_intakeMotor.set(-1), () -> m_intakeMotor.set(0));

    new JoystickButton(m_joystick, 11).whileTrue(new RepeatCommand(new InstantCommand(() -> m_conveyorMotor.set(1))));
    new JoystickButton(m_joystick, 9).whileTrue(new RepeatCommand(new InstantCommand(() -> m_conveyorMotor.set(-1))));
    new JoystickButton(m_joystick, 11).onFalse(new InstantCommand(() -> m_conveyorMotor.set(0)));
    new JoystickButton(m_joystick, 9).onFalse(new InstantCommand(() -> m_conveyorMotor.set(0)));
    
    joystickInstantToggle(3, () -> m_leftAngleMotor.set(.15), () -> m_leftAngleMotor.set(0));
    joystickInstantToggle(4, () -> m_leftAngleMotor.set(-.15), () -> m_leftAngleMotor.set(0));
    joystickInstantToggle(7, () -> {m_leftOneShootMotor.setSpeed(m_leftAimTarget.getRPMSelection());m_rightOneShootMotor.setSpeed(m_rightAimTarget.getRPMSelection());/*m_leftPacerMotor.set(-1);m_rightPacerMotor.set(-1);*/}, () -> {m_leftOneShootMotor.setSpeed(RPM.of(0));m_rightOneShootMotor.setSpeed(RPM.of(0));/*m_rightPacerMotor.set(0);m_leftPacerMotor.set(0);*/});
    // joystickInstantToggle(12, () -> m_ropeMotor.set(.3), () -> m_ropeMotor.set(0));
    // joystickInstantToggle(10, () -> m_ropeMotor.set(-.3), () -> m_ropeMotor.set(0));
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
        "Climb", new InstantCommand(() -> System.out.println("surely there is a climb design")));
    
        NamedCommands.registerCommand(
        "Hood Down", new InstantCommand(() -> System.out.println("surely there is a hood down")));
    NamedCommands.registerCommand("Shoot", m_autoLeftAimTarget.alongWith(m_autoRightAimTarget));
    NamedCommands.registerCommand("Aim", m_autoLeftAimTurret.alongWith(m_autoRightAimTurret));
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> {m_conveyorMotor.set(1.0); m_feederMotor.set(-1);m_intakeMotor.set(1.0);}));
    NamedCommands.registerCommand("Process", new InstantCommand(() -> {m_conveyorMotor.set(1.0);m_feederMotor.set(-1);}));
    NamedCommands.registerCommand("Deploy Hopper", new InstantCommand(() -> System.out.println("surely there is a Deploy Hopper")));
    NamedCommands.registerCommand("Chomp", new InstantCommand(() -> System.out.println("surely there is a Chomp")));
  }

  public void stopMotors(){
      m_intakeMotor.set(0);
      
      m_leftAimingMotor.set(0);
      m_rightAimingMotor.set(0);
      m_conveyorMotor.set(0);
      m_feederMotor.set(0);
      m_rightAngleMotor.set(0);
      m_leftAngleMotor.set(0);
  }

  public static void resetQuestPose(Pose2d pose){
    System.out.println("I'm reseting the " + m_questNav);
    m_questNav.resetPose(pose);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // var alliance = DriverStation.getAlliance();
    return autoChooser.get().handleInterrupt(
        () -> {
                DriveCommands.joystickDrive(drive, 0, 0, 0);
                
        });
    }
}
