// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveSubsystem;

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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.VisionConstants;
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
  private final AHRS m_navX;

  private Field2d m_field = new Field2d();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;
  private int count = 0;
  private int tvCount = 0;
  private final PIDController m_rollPidController = new PIDController(0.0055, 0.00008, 0.0007); // 3/9 kp 0.005  2/15 kp 0.005 kd 0.001  1/21 ki:0.0055 kd: 0.0025
  private final PIDController m_rotPidController = new PIDController(0.01, 0.000, 0.000);
  private final PIDController m_rotVisionPidController = new PIDController(0.020, 0.0, 0.002);
  private final PIDController m_strafeVisionPidController = new PIDController(0.040, 0.0, 0.0);
  private final Trigger m_slowDriveButton;

  // followTrajectoryCommand PID variables
  private double m_kpX;
  private double m_kiX;
  private double m_kdX;

  private double m_kpY;
  private double m_kiY;
  private double m_kdY;

  private double m_kpR;
  private double m_kiR;
  private double m_kdR;

  NetworkTable table; 
  NetworkTableEntry tx; 
  NetworkTableEntry ty; 
  NetworkTableEntry ta; 
  NetworkTableEntry tv; 

  double x = 0.0; 
  double y = 0.0; 
  double area = 0.0; 
  double v = 0.0; 
  double rotVisionSetpoint = 0.0; 
  double rotVisionPieceOffset = 0.0;
  double strafeVisionSetpoint = 0.0;
  double strafeVisionPieceOffset = 0.0;

  // double target = 0.0;
  // double tLength = 0.0;
  double originaltx = 0.0;
  // double newtx = 0.0;
  double txPast = 0.0;
  double txDelta = 0.0;
  double rotSpeed = 0.0;
  
  double txEstimated = 0.0;

  boolean txRejected = false;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Trigger slowDriveButton) {
    m_navX = new AHRS(SPI.Port.kMXP);
    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_navX.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
    m_slowDriveButton = slowDriveButton;
    m_rotPidController.enableContinuousInput(-180, 180);
    zeroHeading();
    table = NetworkTableInstance.getDefault().getTable("limelight");
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

    SmartDashboard.putNumber("Heading", m_odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("CurrentPitch", m_navX.getRoll());
    // SmartDashboard.putNumber("xTrajSP", xTrajPID.getSetpoint());
    // SmartDashboard.putNumber("xTrajPosErr", xTrajPID.getPositionError());

    // Vision
    SmartDashboard.putNumber("txCurrent", table.getEntry("tx").getDouble(0.0));
    SmartDashboard.putNumber("txPast", txPast);
    SmartDashboard.putBoolean("txRejected", txRejected);
    SmartDashboard.putNumber("txEstimated", txEstimated);
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
    drive(xSpeed, ySpeed, rot, fieldRelative, true, false, 1, false);
  }
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit, true, 1, false);
  }
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, boolean squaredInputs, double maxOutput) {
    drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit, squaredInputs, maxOutput, false);
  }

  // Main drive method
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, boolean squaredInputs, double maxOutput, boolean rotException) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    //Override max output when slowdrive button is pressed
    if (m_slowDriveButton.getAsBoolean()) maxOutput = DriveConstants.kDriveSlow; 

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
      if (!rotException) m_currentRotation = m_rotLimiter.calculate(rot);
      else m_currentRotation = rot;


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
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, m_odometry.getPoseMeters().getRotation())
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
    resetOdometry(new Pose2d(getPose().getTranslation(),new Rotation2d()));
    // m_navX.reset();
  }
  //FYI: Don't use m_navX.calibrate(), the method does nothing
  //Startup Cal takes 20s

  public boolean isCalibrating(){
    return m_navX.isCalibrating();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  // public double getHeading() {
  //   return Rotation2d.fromDegrees(-m_navX.getAngle()).getDegrees();
  // }

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
    double pidOut = MathUtil.clamp(m_rollPidController.calculate(
      m_navX.getRoll(), 0), -DriveConstants.kAutoLevelMaxOutput, DriveConstants.kAutoLevelMaxOutput);
    drive(pidOut, 0, 0, false);

    if (++count %10 == 0) {
        System.out.println("Roll is :" + m_navX.getRoll());
        System.out.println("Pitch is :" + m_navX.getPitch());
        System.out.println("PID Output is: " + pidOut);
    }

    SmartDashboard.putNumber("DrivePidOutput",pidOut);
  }

  public boolean isLevel(){
    return Math.abs(m_navX.getRoll()) < Constants.AutoConstants.kLevelTolerance;
  }

  public boolean driveToAngle (double targetAngle){ 
    boolean atAngle = true;
    double currentAngle = m_navX.getRoll();
    // double currentAngle = m_odometry.;
    if ( currentAngle >= targetAngle) {
        drive(.25, 0, 0, false);  // xSpeed .4
        atAngle = false;
        if (++count %10 == 0) {
          System.out.println("Angle is:" + currentAngle);
        }
    }
    else {
        drive(0, 0, 0, true);
    }
    return atAngle;
  }
    
    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
              this.resetOdometry(transformed.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(
            traj, 
            this::getPose, // Pose supplier
            Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDController(PathConstants.kpXdefault, PathConstants.kiXdefault, PathConstants.kdXdefault), // Forward/Backward X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(PathConstants.kpYdefault, PathConstants.kiYdefault, PathConstants.kdYdefault), // Strafe Y controller (usually the same values as X controller)
            new PIDController(PathConstants.kpRdefault, PathConstants.kiRdefault, PathConstants.kdRdefault), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem

            //TODO: 1 inch undershot Forward/Backward. Increasing Xkp and Xki increases this error
        )
    );
  }

  public void setTrajPID(
    double kpX, double kiX, double kdX,
    double kpY, double kiY, double kdY,
    double kpR, double kiR, double kdR
    ){
    m_kpX = kpX;  m_kpY = kpY;  m_kpR = kpR;
    m_kiX = kiX;  m_kiY = kiY;  m_kiR = kiR;
    m_kdX = kdX;  m_kdY = kdY;  m_kdR = kdR;
  }

  public void setXTrajPID(double kp, double ki, double kd){
    m_kpX = kp;
    m_kiX = ki;
    m_kdX = kd;
  }
  public void setYTrajPID(double kp, double ki, double kd){
    m_kpY = kp;
    m_kiY = ki;
    m_kdY = kd;
  }
  public void setRotTrajPID(double kp, double ki, double kd){
    m_kpR = kp;
    m_kiR = ki;
    m_kdR = kd;
  }
  // Use if PID values are set manually (does not affect default values)
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, boolean isModifiedPID) {
    if (!isModifiedPID){
      m_kpX = PathConstants.kpXdefault;
      m_kiX = PathConstants.kiXdefault;
      m_kdX = PathConstants.kdXdefault;

      m_kpY = PathConstants.kpYdefault;
      m_kiY = PathConstants.kiYdefault;
      m_kdY = PathConstants.kdYdefault;

      m_kpR = PathConstants.kpRdefault;
      m_kiR = PathConstants.kiRdefault;
      m_kdR = PathConstants.kdRdefault;
    }
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
              this.resetOdometry(transformed.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(
            traj, 
            this::getPose, // Pose supplier
            Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDController(m_kpX, m_kiX, m_kdX), // Forward/Backward X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(m_kpY, m_kiY, m_kdY), // Strafe Y controller (usually the same values as X controller)
            new PIDController(m_kpR, m_kiR, m_kdR), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
  }

  // TODO: use profiled pid if needed
  public void TurnToTarget(double X, double Y, double angle, boolean rateLimit, boolean squaredInputs, double maxOutput){
    // double pidOut = MathUtil.clamp(m_rotPidController.calculate(-m_navX.getAngle()%360, angle), -0.30, 0.30);
    double pidOut = MathUtil.clamp(m_rotPidController.calculate(MathUtil.inputModulus(m_odometry.getPoseMeters().getRotation().getDegrees(), -180, 180), angle), -DriveConstants.kmaxPOVturnspeed, DriveConstants.kmaxPOVturnspeed);
    drive(X, Y, pidOut, true, rateLimit, squaredInputs, maxOutput, true); // added rotExeption to keep the driver's SquaredInputs and MaxOutput seperate from PID rotation
  }

  public Command DriveCommand(double speed){
    return new StartEndCommand(
      () -> drive(speed,0,0,false), 
      () -> drive(0,0,0,false),
      this);
  }

  public void setLimelightLEDsOn(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  public void setLimelightLEDsOff(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  public void setVisionOriginaltx(){
    txPast = table.getEntry("tx").getDouble(0.0);
    txRejected = false;
  }

  public double getVisionRotSpeed(){
    if (table.getEntry("tv").getDouble(0.0) == 0.0){
      if (++tvCount %10 == 0) {
        txPast = 0.0;
      }
    } else tvCount = 0; // When tv is 1
    
    // Addresses which target to prioritize using smartTargets
    // double targetHalf = table.getEntry(VisionConstants.tLength).getDouble(0.0) / 2;
    // // rotVisionSetpoint = 0.0;

    // if (originaltx > 0){ // target center to the right (cam relative)
    //   // // newtx = originaltx + targetHalf;
    //   // rotVisionSetpoint = -targetHalf;

    // } else if (originaltx < 0){ // target center to the left (cam relative)
    //   // // newtx = originaltx - targetHalf;
    //   // rotVisionSetpoint = targetHalf;
    // }


    // Addresses Target Switching using singleTarget
    txDelta = table.getEntry("tx").getDouble(0.0) - txPast;
    if ((txDelta > VisionConstants.kDeltaThreshhold) || (txDelta < -VisionConstants.kDeltaThreshhold)){
      if (txRejected = false) {
        txEstimated = txPast;
        txRejected = true;
      }
      rotSpeed = MathUtil.clamp(m_rotVisionPidController.calculate(txEstimated, rotVisionSetpoint), 
      -DriveConstants.maxVisionRotSpeed, DriveConstants.maxVisionRotSpeed);
      txEstimated = txEstimated*0.9; // reduced 10% each loop
    } else {
    txRejected = false;
    rotSpeed = MathUtil.clamp(m_rotVisionPidController.calculate(table.getEntry("tx").getDouble(0.0), rotVisionSetpoint), 
      -DriveConstants.maxVisionRotSpeed, DriveConstants.maxVisionRotSpeed);
      txPast = table.getEntry("tx").getDouble(0.0);
      }

    return rotSpeed;
    
  }

  public double getVisionStrafeSpeed(){
    // double heading = m_odometry.getPoseMeters().getRotation().getDegrees();
    
    // if (heading > DriveConstants.faceBackward - DriveConstants.kRobotHeadingTolerance 
    //   && heading < -DriveConstants.faceBackward + DriveConstants.kRobotHeadingTolerance) {
        return MathUtil.clamp(m_strafeVisionPidController.calculate(table.getEntry("tx").getDouble(0.0), strafeVisionSetpoint),
        -DriveConstants.maxVisionStrafeSpeed, DriveConstants.maxVisionStrafeSpeed);
    // } else return 0;
  }
}
