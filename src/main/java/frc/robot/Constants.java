// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CANIDConstants {
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;
    
    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final int kLeftArmCANId = 5; 
    public static final int kRightArmCANId = 4;

    public static final int kIntakeCANId = 3;

    //Power Distribution Hub (PDH) CAN ID
    public static final int kPdhCanId = 1;

    //Ground Intake
    public static final int kLeftGroundIntakeCANId = 6;
    public static final int kRightGroundIntakeCANId = 7;
    public static final int kGroundIntakeJointCANId = 8; 
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.46;
    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 3.6; // radians per second
    public static final double kMagnitudeSlewRate = 3.6; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 4.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = false;

    // Drivetrain Speeds
    public static final double kDriveMaxOutput = 0.95;  // Field: 0.85 | Arcadia: 0.40
    public static final double kDriveSlow = 0.25; //0.25
    public static final double kmaxPOVturnspeed = 1.0; //0.45
    public static final double kAutoLevelMaxOutput = 0.30;
    // Vision
    public static final double maxVisionRotSpeed = 0.4;
    public static final double maxVisionStrafeSpeed = 1.0; // handled by maxOutput (TODO: Add xExeption param to drive(...))
    public static final double kRobotHeadingTolerance = 1.0; // in degrees


    // Cardinal Directions
    public static final double faceForward = 0;
    public static final double faceBackward = 180;
    public static final double faceLeft = 90;
	  public static final double faceRight = -90;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  // Operator Input
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1; 
    public static final int kTestControllerPort = 2;
    public static final double kDriveDeadband = 0.05;
    public static final double kJoystickDeadband = 0.05;
    public static final double kArmDeadband = 0.02;
    public static final double kIntakeDeadband = 0.02;
    public static final double kIntakeReverseDeadband = 0.10;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double kLevelTolerance = 2.25;  //field tolerance is 2.25 degrees 

    public static final double kDriveAngle = -11;  // Was 14.0

    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();
  }

  public static final class PathConstants{
    public static final double kpXdefault = 1.5;
    public static final double kiXdefault = 0.0;
    public static final double kdXdefault = 0.0;
    
    public static final double kpYdefault = 3.0;
    public static final double kiYdefault = 0.0;
    public static final double kdYdefault = 0.0;
    
    public static final double kpRdefault = 5.0;
    public static final double kiRdefault = 0.0;
    public static final double kdRdefault = 0.0;

    public static final int LVL = 0;
    public static final int NoLVL = 1;
    public static final int grabCube = 2;

  }

  public static final class VisionConstants{
    public static final double kLimelightOffDelay = 3.0;
    public static final String tLength = "thor";
    public static final double kDeltaThreshhold = 4; // in degrees
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ArmConstants{
    // p=.01, i=.001 resulted in inverted elbow
    public static final double kArmP = 0.08; // 0.005
    public static final double kArmI = 0.0; // 0.0002
    public static final double kArmD = 0.0; 
    public static final double kArmFF = 0.0; // 0.0005

    // Arm Speed
    public static final double kArmMinOutput = -1.00; //-0.65
    public static final double kArmMaxOutput = 1.00; //0.65

    // Arm Positions
    public static final double kOffset = 0.306;
    public static final double kCONESTOW = kOffset + 0.011;
    public static final double kLVLONE = kOffset + 0.150;
    public static final double kLVLTWO = kOffset + 0.280;
    public static final double kCONESINGLE = kOffset + 0.325;
    public static final double kLVLTRE = kOffset + 0.365;
    public static final double kAllowedErrAbs = 0.001;

    // Soft Limits
    public static final double kMaxHeightAbs = kLVLTRE + 0.002; // The lower the value, the higher the arm
    public static final double kMinHeightAbs = kOffset - 0.002; // The higher the value, the lower the arm

    // Unused
    public static final double kArmMaxVel = 0;
    public static final double kArmMinVel = 0;
    public static final double kArmMaxAcc = 0;
    public static final double kAllowedErrRelative = 0.25;
    public static final double kMaxHeightRelative = 78;

  }

  public static final class IntakeConstants{
    public static final int kPdhChannel = 0;
    public static final double kIntakeStall = 0.2; // + for cone, - for cube

    public static final double kConePickUp = 0.3;
    public static final double kConeEject = -0.5;

    public static final double kCubePickUp = -0.5;
    public static final double kCubeEject = 0.3;
  }

  public static final class GroundArmConstants{
  // Joint Speed: Positive  is up, negative speed down
  // Intake Speed: Positive is pick-up, negative is eject
  // Joint Encoder: 0 = arm inside chassis, negative = down
    public static final double kOutP = 0.06;
    public static final double kInP = 0.06;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static final double kMaxJointOutSpeed = -0.30; // was -0.30
    public static final double kMaxJointInSpeed = 0.40; // was 0.40

    public static final double kOutPosition = -13.35; // -13.1
    public static final double kInPosition = 0.0;

    public static final double kShootPosition = -7.0; // setpoint (executed from outPosition)
    
    public static final double kAutoHighShootPosition = -6.5;
    public static final double kAutoMidShootPosition = -7.5;
    public static final double kAutoFarShootPosition = -9.0;

    public static final double kJointTolerance = 0.10;
    public static final double kShootJointTolerance = 0.05;

    public static final double kPIDDeadband = 0.001;
  
    // Intake
    public static final double kAutoIntakePickUp = 0.6;
    public static final double kIntakePickUp = 0.4; // 0.5
    public static final double kIntakeReverse = -0.5;
    public static final double kIntakeShoot = -1.0;
    public static final double kIntakeShootL2 = -0.5;
    
  /**
   * How many amps the ground intake can take while moving
   */
  public static final int GROUND_JOINT_CURRENT_LIMIT_A = 25;

  public static final double kOffVelocity = 0.1; // between -0.1 and 0.1 RPM for 100ms to set atPosition = true
  public static final double kMaxManualGroundJointSpeed = 0.15;
public static final double kDefaultSpeed = 0.15; // 0.10
  }
 /**
   * How many amps the arm motor can use.
   */
  public static final int ARM_CURRENT_LIMIT_A = 20; //originally 10 for testing

  /**
   * Percent output to run the arm up/down at
   */
  public static final double ARM_OUTPUT_POWER = 0.4;

  /**
   * How many amps the intake can use while picking up
   */
  public static final int INTAKE_CURRENT_LIMIT_A = 10; // Everybot was 25

  /**
   * How many amps the intake can use while holding
   */
  public static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for intaking
   */
  public static final double INTAKE_OUTPUT_POWER = 1.0;

  /**
   * Percent output for holding
   */
  public static final double INTAKE_HOLD_POWER = 0.07;

  /**
   * Time to extend or retract arm in auto
   */
  public static final double ARM_EXTEND_TIME_S = 2.0;
}
