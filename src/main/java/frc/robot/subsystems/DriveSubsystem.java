// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import frc.robot.Constants;
import edu.wpi.first.util.WPIUtilJNI;
// import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      CANIDConstants.kFrontLeftDrivingCanId,
      CANIDConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      CANIDConstants.kFrontRightDrivingCanId,
      CANIDConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      CANIDConstants.kRearLeftDrivingCanId,
      CANIDConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      CANIDConstants.kRearRightDrivingCanId,
      CANIDConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final AHRS m_navX = new AHRS(SPI.Port.kMXP);

  private Field2d m_field = new Field2d();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_navX.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  private int count = 0;
  private final PIDController m_rollPidController = new PIDController(0.005, 0.000, 0.001); // 1/21 ki:0.0055 kd: 0.0025
  private final PIDController m_rotPidController = new PIDController(0.01, 0.000, 0.000); // TODO (requires bot): values need testing

  // private final PIDController xTrajPID = new PIDController(0.05, 0, 0);


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putData(m_field);
    SmartDashboard.putBoolean("Navx is calibrating: ", m_navX.isCalibrating());
    m_odometry.update(
        Rotation2d.fromDegrees(-m_navX.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

        m_field.setRobotPose(m_odometry.getPoseMeters());

    SmartDashboard.putNumber("Heading", m_navX.getAngle());
    // SmartDashboard.putNumber("xTrajSP", xTrajPID.getSetpoint());
    // SmartDashboard.putNumber("xTrajPosErr", xTrajPID.getPositionError());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_navX.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    drive(xSpeed, ySpeed, rot, fieldRelative, false, false, 1, false);  //TODO - Do we want rateLimit for auto?
  }
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit, false, 1, false);
  }
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, boolean squaredInputs, double maxOutput) {
    drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit, squaredInputs, maxOutput, false);
  }

  // Main drive method
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, boolean squaredInputs, double maxOutput, boolean rotException) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (squaredInputs) {
      xSpeed = Math.copySign(xSpeed*xSpeed, xSpeed);
      ySpeed = Math.copySign(ySpeed*ySpeed, ySpeed);
      if (!rotException) {rot = Math.copySign(rot*rot, rot);}
    }

    xSpeed *= maxOutput;
    ySpeed *= maxOutput;
    if (!rotException) {rot *= maxOutput;}

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_navX.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_navX.reset();
  }
  //FYI: Don't use m_navX.calibrate(), the method does nothing
  //Startup Cal takes 20s


  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_navX.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navX.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  //Charge Station Autos
  public void driveToLevel(){
    double pidOut = MathUtil.clamp(m_rollPidController.calculate(m_navX.getRoll(), 0), -0.30, 0.30);
    drive(-pidOut, 0, 0, false);
    if (++count %10 == 0) {
        System.out.println("Roll is :" + m_navX.getRoll());
        System.out.println("Pitch is :" + m_navX.getPitch());
        System.out.println("PID Output is: " + pidOut);
    }
    // SmartDashboard.putNumber("DriveSetPnt",0);
    SmartDashboard.putNumber("DrivePidInput",m_navX.getRoll());
    SmartDashboard.putNumber("DrivePidOutput",pidOut);
  }

  public boolean isLevel(){
    return Math.abs(m_navX.getRoll()) < Constants.AutoConstants.kLevelTolerance;
  }

  public boolean driveToAngle (double angle){
    boolean atAngle = true;
    double currentAngle = m_navX.getRoll();
    // TODO (requires bot): Angle could be positive or negative, need to handle both
    // Should be positive if driving forward, negative if driving backward onto the power station ramp
    if ( currentAngle <= angle) {
        drive(.20, 0, 0, true);
        atAngle = false;
        if (++count %10 == 0) {
          System.out.println("Angle is:" + currentAngle);
        }
        SmartDashboard.putNumber("CurrentRoll", currentAngle);
    }
    else {
        drive(0, 0, 0, true);
    }
    return atAngle;
  }
    
    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    // TODO(requires bot) - Adjust p for x and y
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(
            traj, 
            this::getPose, // Pose supplier
            Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDController(1.5, 0, 0), // Forward/Backward X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(2.0, 0.5, 0), // Strafe Y controller (usually the same values as X controller)
            new PIDController(5.0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem

            //1 inch undershot Forward/Backward. Increasing Xkp and Xki increases this error
            //TODO (requires bot) - Adjust max vel and acc constaints
        )
    );
  }
  // TODO: use profiled pid if needed
  // TODO: add %180 and other functionality to ensure rot takes the shortest path
  public void TurnToTarget(double X, double Y, double angle, boolean rateLimit, boolean squaredInputs, double maxOutput){
    double pidOut = MathUtil.clamp(m_rotPidController.calculate(-m_navX.getAngle()%360, angle), -0.30, 0.30);
    drive(X, Y, pidOut, true, rateLimit, squaredInputs, maxOutput, true); // added rotExeption to keep the driver's SquaredInputs and MaxOutput seperate from PID rotation
  }
}
