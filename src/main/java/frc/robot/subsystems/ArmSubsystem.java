package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.RelativeEncoder;

import java.util.EnumMap;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{
    public enum armPositions{
        LVLONE, 
        LVLTWO, 
        LVLTRE,
        HOME,
        CONESTOW,
        CONESINGLE
    }
        
    private final CANSparkMax armMotorLeft = new CANSparkMax(Constants.CANIDConstants.kLeftArmCANId, MotorType.kBrushless);
    private final CANSparkMax armMotorRight = new CANSparkMax(Constants.CANIDConstants.kRightArmCANId, MotorType.kBrushless);

    // private final SparkMaxPIDController armPID = armMotorLeft.getPIDController(); 

    // private final ProfiledPIDController armPID = 
    //     new ProfiledPIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD, 
    //     new Constraints(ArmConstants.kArmMaxVel, ArmConstants.kArmMaxAcc));
    
    // private final RelativeEncoder armEncoder = armMotorLeft.getEncoder(); 
    
    private final DutyCycleEncoder armAbsEncoder = new DutyCycleEncoder(0);

    // EnumMap<armPositions, Double> mapRelative = new EnumMap<>(armPositions.class); 

    EnumMap<armPositions, Double> mapAbs = new EnumMap<>(armPositions.class);

    private final PIDController m_AbsPidController = new PIDController(0.0, 0.0, 0.0); //i was 0.2 | p is set below

    public ArmSubsystem(){
        armMotorLeft.setInverted(true);
        armMotorRight.follow(armMotorLeft, true); 
 
        // Adding elements to the Map
        // using standard put() method
        // mapRelative.put(armPositions.LVLONE, 23.0);
        // mapRelative.put(armPositions.LVLTWO, 60.0);
        // mapRelative.put(armPositions.LVLTRE, 77.5);
        // mapRelative.put(armPositions.HOME, 0.0);
        // mapRelative.put(armPositions.CONESTOW, 10.0);

        mapAbs.put(armPositions.HOME, ArmConstants.kOffset);
        mapAbs.put(armPositions.CONESTOW, ArmConstants.kCONESTOW);
        mapAbs.put(armPositions.LVLONE, ArmConstants.kLVLONE);
        mapAbs.put(armPositions.LVLTWO, ArmConstants.kLVLTWO);
        mapAbs.put(armPositions.CONESINGLE, ArmConstants.kCONESINGLE); // Single Substation
        mapAbs.put(armPositions.LVLTRE, ArmConstants.kLVLTRE); //At hard stop:
        // .235
        
        // armPID.setP(ArmConstants.kArmP);
        // armPID.setI(ArmConstants.kArmI);
        // armPID.setD(ArmConstants.kArmD);
        // armPID.setFF(ArmConstants.kArmFF);
        // armPID.setOutputRange(ArmConstants.kArmMinOutput, ArmConstants.kArmMaxOutput);

        armMotorLeft.setIdleMode(IdleMode.kBrake);
        armMotorRight.setIdleMode(IdleMode.kBrake);

        armMotorLeft.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT_A);
        armMotorRight.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT_A);
    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    // int smartMotionSlot = 0;
    // armPID.setSmartMotionMaxVelocity(ArmConstants.kArmMaxVel, smartMotionSlot); // not used
    // armPID.setSmartMotionMinOutputVelocity(ArmConstants.kArmMinVel, smartMotionSlot); // not used
    // armPID.setSmartMotionMaxAccel(ArmConstants.kArmMaxAcc, smartMotionSlot);
    // armPID.setSmartMotionAllowedClosedLoopError(ArmConstants.kAllowedErrRelative, smartMotionSlot);

    // SmartDashboard.putNumber("Arm base position",baseEncoderPosition);
    // SmartDashboard.putNumber("Arm Enc", armMotorLeft.getEncoder().getPosition());

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Relative Enc", armMotorLeft.getEncoder().getPosition());
        SmartDashboard.putNumber("ArmABS Absolute", armAbsEncoder.getAbsolutePosition()); 
        // SmartDashboard.putNumber("ArmABS Offset", armAbsEncoder.getPositionOffset());  
        //   Might use global that is set by drive periodic to indicate if driving too fast.
    }

    // public void raiseArmRelative(armPositions position){
    //     if (((armEncoder.getPosition() < 0) && (position == armPositions.HOME)) ||
    //         ((armEncoder.getPosition() > ArmConstants.kMaxHeightRelative) && (position == armPositions.LVLTRE))) {
    //         armMotorLeft.set(0);
    //         return;
    //     }
    //     double ref = mapRelative.get(position);
    //     SmartDashboard.putNumber("Arm Target Pos", ref);
    //     armPID.setReference(ref, CANSparkMax.ControlType.kPosition);
    //     armPID.setFeedbackDevice(armEncoder);
    // }

    public void raiseArmAbs(armPositions position){
        if (((armAbsEncoder.getAbsolutePosition() > ArmConstants.kMinHeightAbs) && (position == armPositions.HOME)) ||
            ((armAbsEncoder.getAbsolutePosition() < ArmConstants.kMaxHeightAbs) && (position == armPositions.LVLTRE))) {
            armMotorLeft.set(0);
            return;
        }

        // For LVLTRE, LVLTWO, and HOME
        switch (position) {
            case LVLTRE:
            case LVLTWO:
            case HOME:
                m_AbsPidController.setP(11.0);
                break;
        // For LVLONE and CONESTOW
            case CONESTOW:
            case LVLONE:
            default:
                m_AbsPidController.setP(9.0);
                break;
        }
        double ref = mapAbs.get(position);

        double pidOut = MathUtil.clamp(
            m_AbsPidController.calculate(armAbsEncoder.getAbsolutePosition(),ref),
            Constants.ArmConstants.kArmMinOutput, Constants.ArmConstants.kArmMaxOutput);
            
        SmartDashboard.putNumber("Arm Abs Target Pos", ref);
        armMotorLeft.set(-pidOut);
        // TODO: Add a new armPosition that reads a value from the smart dashboard and moves arm to that position.
    }

    public void raiseArm(double speed){
        if (((armAbsEncoder.getAbsolutePosition() > ArmConstants.kMinHeightAbs) && (speed < 0)) ||
            ((armAbsEncoder.getAbsolutePosition() < ArmConstants.kMaxHeightAbs) && (speed > 0))) {
            armMotorLeft.set(0);
            return;
        }
        armMotorLeft.set(speed);
    }

    public void raiseArm(double raiseSpeed, double lowerSpeed){
        double speed = raiseSpeed - lowerSpeed; //positive output to raise arm
        if (((armAbsEncoder.getAbsolutePosition() > ArmConstants.kMinHeightAbs) && (speed < 0)) ||
            ((armAbsEncoder.getAbsolutePosition() < ArmConstants.kMaxHeightAbs) && (speed > 0))) {
            armMotorLeft.set(0);
            return;
        }
        armMotorLeft.set(speed);
    }

    public boolean atPosition(armPositions pos){
        double currentEncoderPosition = armAbsEncoder.getAbsolutePosition();
        return (Math.abs(currentEncoderPosition - mapAbs.get(pos)) < Constants.ArmConstants.kAllowedErrAbs);
    }

}

