// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;

// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.GroundArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.PathPlanner.C1ThreePiece;
import frc.robot.commands.PathPlanner.C1TwoPiece;
import frc.robot.commands.PathPlanner.C3OneCone;
import frc.robot.commands.PathPlanner.C3OneCubeHybrid;
import frc.robot.commands.PathPlanner.C5OneCubeLevel;
import frc.robot.commands.PathPlanner.C7HybridLink;
import frc.robot.commands.PathPlanner.C4C6OneConeLevel;
import frc.robot.commands.PathPlanner.C4C6TwoPieceCS;
import frc.robot.commands.PathPlanner.C9TwoPiece;
import frc.robot.commands.PathPlanner.OneConeScoreSolo;
import frc.robot.commands.PathPlanner.OneCubeScoreSolo;
import frc.robot.commands.PathPlanner.TEST;
import frc.robot.commands.PathPlanner.C7OneCone;
import frc.robot.commands.PathPlanner.C9OneConeShootTwo;
import frc.robot.commands.armCommands.ArmToPosition;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.commands.driveCommands.AutoLevel;
import frc.robot.commands.driveCommands.DriveToLevel;
import frc.robot.commands.groundIntakeCommands.IntakeHandoff;
import frc.robot.commands.navXCommands.ResetGyro;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem.GroundIntake;
import frc.robot.subsystems.GroundIntakeSubsystem.GroundJoint;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  SendableChooser<CommandBase> m_autoChooser = new SendableChooser<>();

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;
  private final ArmSubsystem m_arm; 
  private final IntakeSubsystem m_intake;
  private final GroundIntake m_groundIntake;
  private final GroundJoint m_groundJoint;
  private final LEDs m_LEDs;
  // private final UsbCamera m_cameraUSB;

  // The driver's controller
//   XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final CommandXboxController m_manipulatorController = 
      new CommandXboxController(OIConstants.kManipulatorControllerPort); 
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_testController =
      new CommandXboxController(OIConstants.kTestControllerPort);
      
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // The robot's subsystems
    m_robotDrive = new DriveSubsystem(m_driverController.rightBumper()); //Be careful when pressing this buttton while doing an auto command
    m_arm = new ArmSubsystem(); 
    m_intake = new IntakeSubsystem();
    m_LEDs = new LEDs();
    m_groundIntake = new GroundIntake();
    m_groundJoint = new GroundJoint();
    // m_cameraUSB = CameraServer.startAutomaticCapture();

  //   SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
  //     m_robotDrive::getPose, // Pose2d supplier
  //     m_robotDrive::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
  //     Constants.DriveConstants.kDriveKinematics., // SwerveDriveKinematics //TODO: fix SwerveDriveKinematics
  //     new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
  //     new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
  //     m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
  //     Constants.AutoConstants.AUTO_EVENT_MAP,
  //     true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  //     m_robotDrive // The drive subsystem. Used to properly set the requirements of path following commands
  // );
    
    // m_groundIntake = new GroundIntakeSubsystem(); 
    // m_LEDs.setDefaultCommand(new RunCommand(() -> m_LEDs.setColor(.6), m_LEDs));

    configureAutoCommands();

    addAutoOptions();

    // Configure the button bindings
    configureButtonBindings();

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); // 0 is default, 1 is camMode

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                true,
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true, true,
                Constants.DriveConstants.kDriveMaxOutput),
            m_robotDrive));

    m_intake.setDefaultCommand(
      // The right stick controls the intake speed and direction. 
        new RunCommand(
        
            () -> m_intake.intakeOn(
                MathUtil.applyDeadband(-m_manipulatorController.getRightY(), OIConstants.kIntakeDeadband)), 
                // MathUtil.clamp(MathUtil.applyDeadband(-m_manipulatorController.getRightY(), OIConstants.kIntakeDeadband), -0.7, 1.0)), 
            m_intake));
    
    m_groundIntake.setDefaultCommand(
      new RunCommand(
        
            () -> m_groundIntake.groundIntakeSpeed(
              GroundArmConstants.kDefaultSpeed),
            m_groundIntake));

    // REPLACED IN TELEOP INIT
    m_groundJoint.setDefaultCommand(
      new RunCommand(
            () -> m_groundJoint.groundJointOff(),
            m_groundJoint));

    // m_arm.setDefaultCommand(
    //   new RunCommand(
    //         () -> m_arm.raiseArm(0),
    //         m_arm)
    // );
  }

  // Configure auto options
  private void addAutoOptions() {
    m_autoChooser.setDefaultOption("OneCone ScoreSolo", new OneConeScoreSolo(m_robotDrive, m_arm, m_intake));
    m_autoChooser.addOption("OneCube ScoreSolo", new OneCubeScoreSolo(m_robotDrive, m_arm, m_intake));

    m_autoChooser.addOption("C1 RED TwoPiece", new C1TwoPiece(PathConstants.LVL, true, m_robotDrive, m_arm, m_intake, m_LEDs));
    m_autoChooser.addOption("C1 Default TwoPiece", new C1TwoPiece(PathConstants.LVL, false, m_robotDrive, m_arm, m_intake, m_LEDs));
    m_autoChooser.addOption("C1 TwoPiece NO LVL", new C1TwoPiece(PathConstants.NoLVL, false, m_robotDrive, m_arm, m_intake, m_LEDs));
    // m_autoChooser.addOption("C1 TwoPiece & Cube NO LVL", new C1TwoPiece(PathConstants.grabCube, false, m_robotDrive, m_arm, m_intake, m_LEDs));  // Not tested / dialed-in
    m_autoChooser.addOption("C1 ThreePiece", new C1ThreePiece(PathConstants.LVL, false, m_robotDrive, m_arm, m_intake, m_groundJoint, m_groundIntake, m_LEDs));    // Add Red Option if Charge Station behaves differently
    m_autoChooser.addOption("C1 ThreePiece NO LVL", new C1ThreePiece(PathConstants.NoLVL, false, m_robotDrive, m_arm, m_intake, m_groundJoint, m_groundIntake, m_LEDs));    // Add Red Option if Charge Station behaves differently

    m_autoChooser.addOption("C3 OneCone", new C3OneCone(m_robotDrive, m_arm, m_intake));
    m_autoChooser.addOption("C3 OneCubeHybrid", new C3OneCubeHybrid(m_robotDrive, m_arm, m_intake));

    m_autoChooser.addOption("C4 OR C6 TwoPieceCS", new C4C6TwoPieceCS(true, m_robotDrive, m_arm, m_intake, m_LEDs, m_groundIntake));
    m_autoChooser.addOption("C4 OR C6 OneConeLevel", new C4C6OneConeLevel(m_robotDrive, m_arm, m_intake));
    m_autoChooser.addOption("C5 OneCubeLevel", new C5OneCubeLevel(m_robotDrive, m_arm, m_intake));
    m_autoChooser.addOption("C7 OneCone", new C7OneCone(m_robotDrive, m_arm, m_intake));
    m_autoChooser.addOption("C7 Hybrid Link", new C7HybridLink(m_robotDrive, m_arm, m_intake));
    m_autoChooser.addOption("C9 TwoPiece", new C9TwoPiece(PathConstants.LVL, true, m_robotDrive, m_arm, m_intake, m_LEDs));
    m_autoChooser.addOption("C9 TwoPiece NO LVL", new C9TwoPiece(PathConstants.NoLVL, true, m_robotDrive, m_arm, m_intake, m_LEDs));
    m_autoChooser.addOption("C9 OneConeShootTwo", new C9OneConeShootTwo(m_robotDrive, m_arm, m_intake, m_LEDs));

    m_autoChooser.addOption("PID TEST x & y", new TEST(0, m_robotDrive, m_arm, m_intake, m_LEDs));
    m_autoChooser.addOption("PID TEST rot", new TEST(1, m_robotDrive, m_arm, m_intake, m_LEDs));
    m_autoChooser.addOption("PID TEST all", new TEST(2, m_robotDrive, m_arm, m_intake, m_LEDs));
    m_autoChooser.addOption("PID TEST straight", new TEST(3, m_robotDrive, m_arm, m_intake, m_LEDs));
    SmartDashboard.putData("Auto Mode", m_autoChooser);
  }

  // Path with Events
  private void configureAutoCommands(){
    Constants.AutoConstants.AUTO_EVENT_MAP.put("Intake PickUp", new InstantCommand(() -> m_intake.intakeOn(Constants.IntakeConstants.kConePickUp), m_intake));
    Constants.AutoConstants.AUTO_EVENT_MAP.put("IntakeOff", new InstantCommand(() -> m_intake.intakeOff(), m_intake));
    Constants.AutoConstants.AUTO_EVENT_MAP.put("Arm LVLTRE", new ArmToPosition(m_arm, armPositions.LVLTRE, false).withTimeout(2.0));
    Constants.AutoConstants.AUTO_EVENT_MAP.put("Arm LVLONE", new ArmToPosition(m_arm, armPositions.LVLONE, false).withTimeout(2.0));
    Constants.AutoConstants.AUTO_EVENT_MAP.put("Intake Reverse", new RunCommand(() -> m_intake.intakeOn(Constants.IntakeConstants.kConeEject), m_intake).withTimeout(.25));
    Constants.AutoConstants.AUTO_EVENT_MAP.put("Arm HOME", new ArmToPositionWithEnd(m_arm, armPositions.HOME).withTimeout(2.0));
    Constants.AutoConstants.AUTO_EVENT_MAP.put("LED Cycle", new RunCommand(() -> m_LEDs.cycle()));
    Constants.AutoConstants.AUTO_EVENT_MAP.put("AutoLevel", new AutoLevel(m_robotDrive));

    Constants.AutoConstants.AUTO_EVENT_MAP.put("ArmHOME then GroundIntakeOut and PickUp",
      new SequentialCommandGroup(
        new InstantCommand(() -> m_intake.intakeOff()),
        new ArmToPosition(m_arm, armPositions.HOME, true).withTimeout(1.0), 
        new ParallelCommandGroup(
          new ArmToPosition(m_arm, armPositions.HOME, false).withTimeout(0.6),
          new RunCommand(() -> m_groundJoint.groundJointPosition(GroundArmConstants.kOutPosition), m_groundJoint)
            .until(() -> m_groundJoint.groundJointAtPosition())
            .alongWith(new RunCommand(() -> m_groundIntake.groundIntakeSpeed(GroundArmConstants.kAutoIntakePickUp), m_groundIntake)))));

    Constants.AutoConstants.AUTO_EVENT_MAP.put("ArmHOME then GroundIntakeOut and Fast PickUp",
      new SequentialCommandGroup(
        new InstantCommand(() -> m_intake.intakeOff()),
        new ArmToPosition(m_arm, armPositions.HOME, true).withTimeout(1.0), 
        new ParallelCommandGroup(
          new ArmToPosition(m_arm, armPositions.HOME, false).withTimeout(0.6),
          new RunCommand(() -> m_groundJoint.groundJointPosition(GroundArmConstants.kOutPosition), m_groundJoint)
            .until(() -> m_groundJoint.groundJointAtPosition())
            .alongWith(new RunCommand(() -> m_groundIntake.groundIntakeSpeed(GroundArmConstants.kAutoIntakePickUp + 0.2), m_groundIntake)))));

    Constants.AutoConstants.AUTO_EVENT_MAP.put("GroundIntakeTransferArmUp", 
      new SequentialCommandGroup(
        new IntakeHandoff(m_groundIntake, m_groundJoint, m_intake),
        new ArmToPosition(m_arm, armPositions.LVLTRE)
        .alongWith(new RunCommand(() -> m_intake.intakeOn(-IntakeConstants.kIntakeStall), m_intake)))
      );

      Constants.AutoConstants.AUTO_EVENT_MAP.put("GroundIntakeShootPos", // TODO: optimize shooting pos, Change name
        new RunCommand(() -> m_groundJoint.groundJointPosition(Constants.GroundArmConstants.kAutoFarShootPosition),
          m_groundJoint)
          // .until(() -> m_groundJoint.groundJointAtPosition()).withTimeout(1.5)
      );

      Constants.AutoConstants.AUTO_EVENT_MAP.put("ShootPosL3", 
        new RunCommand(() -> m_groundJoint.groundJointPosition(Constants.GroundArmConstants.kAutoHighShootPosition),
          m_groundJoint)
      );

      Constants.AutoConstants.AUTO_EVENT_MAP.put("ShootPosL2", 
        new RunCommand(() -> m_groundJoint.groundJointPosition(Constants.GroundArmConstants.kAutoMidShootPosition),
          m_groundJoint)
      );

      Constants.AutoConstants.AUTO_EVENT_MAP.put("Shoot",
        new SequentialCommandGroup(
          // new RunCommand(() -> m_groundIntake.groundIntakePickUp(), m_groundIntake).withTimeout(0.10), // 2.95, 5.95 markers on C7
          new RunCommand(() -> m_groundIntake.groundIntakeShoot(), m_groundIntake)
        ));

      Constants.AutoConstants.AUTO_EVENT_MAP.put("ShootToL2",
        new SequentialCommandGroup(
          new RunCommand(() -> m_groundIntake.groundIntakeSpeed(GroundArmConstants.kIntakeShootL2), m_groundIntake)
        ));

      Constants.AutoConstants.AUTO_EVENT_MAP.put("GroundIntakeOut and PickUp",
        new RunCommand(() -> m_groundJoint.groundJointPosition(GroundArmConstants.kOutPosition), m_groundJoint)
          .until(() -> m_groundJoint.groundJointAtPosition())
          .alongWith(new RunCommand(() -> m_groundIntake.groundIntakeSpeed(GroundArmConstants.kAutoIntakePickUp), m_groundIntake)));

      Constants.AutoConstants.AUTO_EVENT_MAP.put("GroundIntakeOut and Fast PickUp",
        new RunCommand(() -> m_groundJoint.groundJointPosition(GroundArmConstants.kOutPosition), m_groundJoint)
          .until(() -> m_groundJoint.groundJointAtPosition())
          .alongWith(new RunCommand(() -> m_groundIntake.groundIntakeSpeed(GroundArmConstants.kAutoIntakePickUp + 0.2), m_groundIntake)));

      Constants.AutoConstants.AUTO_EVENT_MAP.put("GroundJointIn",
        new RunCommand(() -> m_groundJoint.groundJointPosition(GroundArmConstants.kInPosition), m_groundJoint));

      Constants.AutoConstants.AUTO_EVENT_MAP.put("GroundIntake PickUp",
        new RunCommand(() -> m_groundIntake.groundIntakePickUp(), m_groundIntake));
  }

  public void updateRobotForTeleop() {
    m_groundJoint.setDefaultCommand(
      new RunCommand(() -> m_groundJoint.groundJointPosition(GroundArmConstants.kInPosition),
                     m_groundJoint));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {


    // DRIVER 1

    // Zero Heading
    m_driverController.leftStick().onTrue(new ResetGyro(m_robotDrive));

    // Prevents Movement
    m_driverController.povDown().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // Charge Station Auto Level
    m_driverController.povUp().whileTrue(new DriveToLevel(m_robotDrive)); // Auto Level while ON Charge Station
    m_driverController.povLeft().whileTrue(new AutoLevel(m_robotDrive));  // Full AutoLevel Sequence while OFF Charge Station

    // Main Intake Drop-off
    // Intake deadband to prevent accidental activation
    m_driverController.rightTrigger(OIConstants.kIntakeReverseDeadband).whileTrue(new RunCommand(() -> 
      m_intake.operatorReverse(m_manipulatorController.getRightY(), 
      m_driverController.getRightTriggerAxis())));

    // POV Rotation
    m_driverController.b().whileTrue( new RunCommand (
          () -> m_robotDrive.TurnToTarget(
              true,
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              Constants.DriveConstants.faceRight,
              true, true,
              Constants.DriveConstants.kDriveMaxOutput),
          m_robotDrive));
    m_driverController.x().whileTrue( new RunCommand (
          () -> m_robotDrive.TurnToTarget(
              true,
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              Constants.DriveConstants.faceLeft,
              true, true,
              Constants.DriveConstants.kDriveMaxOutput),
          m_robotDrive));
    m_driverController.y().whileTrue( new RunCommand (
          () -> m_robotDrive.TurnToTarget(
              true,
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              Constants.DriveConstants.faceForward,
              true, true,
              Constants.DriveConstants.kDriveMaxOutput),
          m_robotDrive));
    m_driverController.a().whileTrue( new RunCommand (
          () -> m_robotDrive.TurnToTarget(
              true,
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              Constants.DriveConstants.faceBackward,
              true, true,
              Constants.DriveConstants.kDriveMaxOutput),
          m_robotDrive));

    // Toggle for field oriented vs robot oriented
    // When right stick pressed down, run the robot oriented drive.
    // When right stick pressed down again, end the robot oriented drive and run default drive, which is field oriented drive
    // NOT USED IN MATCH PLAY
    m_driverController.rightStick().toggleOnTrue( new RunCommand (
          () -> m_robotDrive.drive(
              true,
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
              false, true, true,
              Constants.DriveConstants.kDriveMaxOutput),
          m_robotDrive));

      // Allign with Cone Node by Rotation
      m_driverController.start().whileTrue(new RunCommand(
          () -> m_robotDrive.drive(
            false,
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              m_robotDrive.getVisionRotSpeed(),
              true, true, true,
              Constants.DriveConstants.kDriveMaxOutput,
              true),
            m_robotDrive)
              .beforeStarting(new SequentialCommandGroup(
                new RunCommand(() -> m_robotDrive.setLimelightLEDsOn()).withTimeout(0.1), 
                new InstantCommand(() -> m_robotDrive.setVisionOriginaltx())))
              .handleInterrupt(() -> m_robotDrive.setLimelightLEDsOff()

              // .handleInterrupt(() -> new SequentialCommandGroup(
              //   new WaitCommand(VisionConstants.kLimelightOffDelay),
              //   new InstantCommand(() -> m_robotDrive.setLimelightLEDsOff()))
      ));

      // Allign with Cone Node by Strafe
      m_driverController.back().whileTrue(new RunCommand(
          () -> m_robotDrive.TurnToTarget(
              false,
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -m_robotDrive.getVisionStrafeSpeed(),
              Constants.DriveConstants.faceBackward,
              true, true,
              Constants.DriveConstants.kDriveMaxOutput),
            m_robotDrive)
              .beforeStarting(new InstantCommand(() -> m_robotDrive.setLimelightLEDsOn()))
              // .handleInterrupt(() -> m_robotDrive.setLimelightLEDsOff()

              // .handleInterrupt(() -> new SequentialCommandGroup(
              //   new WaitCommand(VisionConstants.kLimelightOffDelay),
              //   new InstantCommand(() -> m_robotDrive.setLimelightLEDsOff()))
      );



    // DRIVER 2

    // LEDs
    m_manipulatorController.leftBumper().onTrue(new InstantCommand(() -> m_LEDs.setPiece(), m_LEDs));
    m_manipulatorController.back().onTrue(new InstantCommand(() -> m_LEDs.off(), m_LEDs))
      .debounce(0.2).onTrue(new RunCommand(() -> m_LEDs.flashing(m_LEDs.PURPLE), m_LEDs))
      .debounce(1.0).onTrue(new RunCommand(() -> m_LEDs.cycle(), m_LEDs));

    // Arm Positions
    m_manipulatorController.povDown().onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.HOME));
    m_manipulatorController.povLeft().onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.LVLONE));
    m_manipulatorController.povUp().onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.LVLTWO));
    m_manipulatorController.povRight().onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.LVLTRE));
    m_manipulatorController.rightTrigger().onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.CONESTOW)
      .alongWith(new InstantCommand(() -> m_LEDs.off())));
    m_manipulatorController.rightBumper().onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.CONESINGLE));
    // m_manipulatorController.rightBumper().whileTrue(new RunCommand(() -> m_intake.intakeOn(1.0), m_intake));

    // Main Intake
    m_manipulatorController.rightStick().whileTrue( new RunCommand(
        () -> m_intake.intakeStall(
            -m_manipulatorController.getRightY(),
            Constants.IntakeConstants.kIntakeStall),
        m_intake));

    // Ground Intake
      //Down and Intake Pickup
    m_manipulatorController.a().whileTrue(
        new RunCommand(() -> m_groundJoint.groundJointPosition(Constants.GroundArmConstants.kOutPosition), m_groundJoint)
        .alongWith(new RunCommand(() -> m_groundIntake.groundIntakePickUp(), m_groundIntake)));
      //Handoff
    m_manipulatorController.start().onTrue(new IntakeHandoff(m_groundIntake, m_groundJoint, m_intake));
      //Intake PickUp
    m_manipulatorController.x().whileTrue(new RunCommand(() -> m_groundIntake.groundIntakePickUp(), m_groundIntake));
      //Intake Reverse
    m_manipulatorController.y().whileTrue(new RunCommand(() -> m_groundIntake.groundIntakeShoot(), m_groundIntake));
      //Down
    m_manipulatorController.b().onTrue(new RunCommand(() -> m_groundJoint.groundJointPosition(GroundArmConstants.kOutPosition), m_groundJoint));
      //Shoot Position
    m_manipulatorController.leftTrigger().whileTrue(new RunCommand(() -> m_groundJoint.groundJointPosition(GroundArmConstants.kShootPosition), m_groundJoint));
    //TODO: Add setpoint that +/- "1" depending on staring pos of groundJoint

    // Arm Manual Control
    m_manipulatorController.leftStick()
    .toggleOnTrue(new ParallelCommandGroup(
        new RunCommand(
          () -> m_arm.raiseArm(
              -MathUtil.applyDeadband(m_manipulatorController.getLeftY()*Constants.ArmConstants.kArmMaxOutput, OIConstants.kArmDeadband)),
          m_arm)
          .handleInterrupt(() -> m_arm.raiseArm(0))
          .handleInterrupt(() -> m_LEDs.setColor(m_LEDs.OFF)),
        new RunCommand(
          () -> m_LEDs.flashing(m_LEDs.WHITE, 10), m_LEDs)))

    // Ground Joint Manual Control
      .debounce(0.5)
      .toggleOnTrue(new ParallelCommandGroup(
        new RunCommand(
          () -> m_groundJoint.groundJointEmergencyControl(
              MathUtil.applyDeadband(-m_manipulatorController.getLeftY()*GroundArmConstants.kMaxManualGroundJointSpeed, OIConstants.kIntakeDeadband)),
          m_groundJoint)
          .handleInterrupt(() -> m_LEDs.setColor(m_LEDs.OFF)),
        new RunCommand(
          () -> m_LEDs.flashing(m_LEDs.WHITE, 5), m_LEDs)));     


    // Test Controller

    // Joint Speed: Positive  is up, negative speed down
    m_testController.rightStick().toggleOnTrue(new RunCommand(() -> 
      m_groundJoint.groundJointSpeed(MathUtil.applyDeadband(-m_testController.getRightY()*GroundArmConstants.kMaxManualGroundJointSpeed, OIConstants.kIntakeDeadband)), m_groundJoint));

    // Intake Speed: Positive is pick-up, negative is eject
    m_testController.leftStick().toggleOnTrue(new RunCommand(() -> 
      m_groundIntake.groundIntakeSpeed(MathUtil.applyDeadband(m_testController.getLeftY(), OIConstants.kIntakeDeadband)), m_groundIntake));

    // Zero Encoder
    m_testController.a().onTrue(new InstantCommand(() -> m_groundJoint.zeroEncoder(), m_groundJoint));
    
    // String TestPathName = new String("Cone Score 3"); 
    // PathPlannerTrajectory m_coneScore1 = PathPlanner.loadPath(TestPathName, new PathConstraints(.85, .5));
    // m_driverController.povRight().toggleOnTrue(m_robotDrive.followTrajectoryCommand(m_coneScore1, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
