// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import java.util.List;
// import edu.wpi.first.wpilibj.PS4Controller.Button;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.armCommands.ArmToPosition;
import frc.robot.commands.driveCommands.AutoLevel;
import frc.robot.commands.driveCommands.DriveToAngle;
import frc.robot.commands.driveCommands.DriveToLevel;
import frc.robot.commands.driveCommands.PathPlanner.OnePieceAuto;
import frc.robot.commands.driveCommands.PathPlanner.ForwardPathTest;
import frc.robot.commands.driveCommands.PathPlanner.ReversePathTest;
import frc.robot.commands.navXCommands.ResetGyro;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ArmSubsystem.armPositions;

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

  // The driver's controller
//   XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final CommandXboxController m_manipulatorController = 
      new CommandXboxController(OIConstants.kManipulatorControllerPort); 
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
      
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // The robot's subsystems
    m_robotDrive = new DriveSubsystem();
    m_arm = new ArmSubsystem(); 
    m_intake = new IntakeSubsystem();

    addAutoOptions();

    // Configure the button bindings
    configureButtonBindings();

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true, true,
                Constants.DriveConstants.kDriveMaxOutput),
            m_robotDrive));
      
    m_driverController.rightBumper().whileTrue(new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true, true,
                Constants.DriveConstants.kDriveSlow),
            m_robotDrive));

      // Toggle for field oriented vs robot oriented
      // When right stick pressed down, run the robot oriented drive.
      // When right stick pressed down again, end the robot oriented drive and run default drive, which is field oriented drive
      m_driverController.rightStick().toggleOnTrue( new RunCommand (
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false, true, true,
                Constants.DriveConstants.kDriveMaxOutput),
            m_robotDrive));

      // POV Rotation
      m_driverController.b().whileTrue( new RunCommand (
            () -> m_robotDrive.TurnToTarget(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                Constants.DriveConstants.faceRight,
                true, true,
                Constants.DriveConstants.kDriveMaxOutput),
            m_robotDrive));
      m_driverController.x().whileTrue( new RunCommand (
            () -> m_robotDrive.TurnToTarget(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                Constants.DriveConstants.faceLeft,
                true, true,
                Constants.DriveConstants.kDriveMaxOutput),
            m_robotDrive));
      m_driverController.y().whileTrue( new RunCommand (
            () -> m_robotDrive.TurnToTarget(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                Constants.DriveConstants.faceForward,
                true, true,
                Constants.DriveConstants.kDriveMaxOutput),
            m_robotDrive));
      m_driverController.a().whileTrue( new RunCommand (
            () -> m_robotDrive.TurnToTarget(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                Constants.DriveConstants.faceBackward,
                true, true,
                Constants.DriveConstants.kDriveMaxOutput),
            m_robotDrive));
            
    m_arm.setDefaultCommand(
      // The left stick controls moving the arm in and out. 
        new RunCommand(
        
            () -> m_arm.raiseArm(
                MathUtil.applyDeadband(m_driverController.getRightTriggerAxis()*Constants.ArmConstants.kArmMaxOutput, OIConstants.kArmDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis()*Constants.ArmConstants.kArmMaxOutput, OIConstants.kArmDeadband)), 
            m_arm));

        m_manipulatorController.leftStick().toggleOnTrue( new RunCommand(
            () -> m_arm.raiseArm(
                -MathUtil.applyDeadband(m_manipulatorController.getLeftY()*Constants.ArmConstants.kArmMaxOutput, OIConstants.kArmDeadband)),
            m_arm));

    m_intake.setDefaultCommand(
      // The right stick controls the intake speed and direction. 
        new RunCommand(
        
            () -> m_intake.intakeOn(
                -m_manipulatorController.getRightY()), 
                // MathUtil.applyDeadband(-m_manipulatorController.getRightY(), OIConstants.kIntakeDeadband)), 
            m_intake));

        // m_driverController.leftBumper().toggleOnTrue( new RunCommand(
        //     () -> m_intake.intakeOn(
        //         -1), //cube speed
        //     m_intake));

        // m_driverController.rightBumper().toggleOnTrue( new RunCommand(
        //     () -> m_intake.intakeOn(
        //         1), //cone speed
        //     m_intake));
                      
  }

  private void addAutoOptions() {
    m_autoChooser.setDefaultOption("OnePieceAuto", new OnePieceAuto(m_robotDrive, m_arm, m_intake));
    SmartDashboard.putData("Auto Mode", m_autoChooser);
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
    m_manipulatorController.x().whileTrue(new RunCommand(() -> m_robotDrive.setX(),m_robotDrive));  //Prevents Movement

    m_manipulatorController.leftTrigger().whileTrue(new DriveToLevel(m_robotDrive)); //On Charge Station

    m_manipulatorController.leftBumper().whileTrue(new DriveToAngle(m_robotDrive));  //Off Charge Station

    m_manipulatorController.y().toggleOnTrue(new AutoLevel(m_robotDrive));  //Command Group

    m_manipulatorController.b().toggleOnTrue(new ForwardPathTest(m_robotDrive)); 

    m_manipulatorController.a().toggleOnTrue(new ReversePathTest(m_robotDrive));

    // m_driverController.povDown().whileTrue(new ResetGyro(m_robotDrive));
    m_driverController.leftStick().toggleOnTrue(new ResetGyro(m_robotDrive));

    
    m_manipulatorController.povDown().onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.HOME)); // TODO: enable quick cancelling of these commands
    m_manipulatorController.povLeft().onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.LVLONE));
    m_manipulatorController.povUp().onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.LVLTWO));
    m_manipulatorController.povRight().onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.LVLTRE));
    m_manipulatorController.rightTrigger().onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.CONESTOW));
  
    // String pathName = new String("Leave Community 3"); 
    String pathName1 = new String("Leave Community 3"); 
    PathPlannerTrajectory m_leaveCommunity = PathPlanner.loadPath(pathName1, new PathConstraints(.85, .5));
    m_driverController.povDown().toggleOnTrue(m_robotDrive.followTrajectoryCommand(m_leaveCommunity, true));

    String pathName2 = new String("Cone Score 3"); 
    PathPlannerTrajectory m_coneScore1 = PathPlanner.loadPath(pathName2, new PathConstraints(.85, .5));
    m_driverController.povRight().toggleOnTrue(m_robotDrive.followTrajectoryCommand(m_coneScore1, true));


    // m_manipulatorController.().whileTrue(new ) //TODO: add button to prevent from running

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
