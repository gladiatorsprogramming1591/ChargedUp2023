package frc.robot.subsystems.MainIntakeSubsystem; 

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

import com.revrobotics.CANSparkMax.IdleMode;

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
    
    private final DutyCycleEncoder armAbsEncoder = new DutyCycleEncoder(0);

    double m_speed = 0.0;

    EnumMap<armPositions, Double> mapAbs = new EnumMap<>(armPositions.class);

    private final PIDController m_AbsPidController = new PIDController(0.0, 0.0, 0.0); //i was 0.2 | p is set below

    public ArmSubsystem(){
        armMotorLeft.setInverted(true);
        armMotorRight.follow(armMotorLeft, true); 

        mapAbs.put(armPositions.HOME, ArmConstants.kOffset);
        mapAbs.put(armPositions.CONESTOW, ArmConstants.kCONESTOW);
        mapAbs.put(armPositions.LVLONE, ArmConstants.kLVLONE);
        mapAbs.put(armPositions.LVLTWO, ArmConstants.kLVLTWO);
        mapAbs.put(armPositions.CONESINGLE, ArmConstants.kCONESINGLE); // Single Substation
        mapAbs.put(armPositions.LVLTRE, ArmConstants.kLVLTRE); //At hard stop:
        
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
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Relative Enc", armMotorLeft.getEncoder().getPosition());
        SmartDashboard.putNumber("ArmABS Absolute", armAbsEncoder.getAbsolutePosition()); 
        SmartDashboard.putNumber("Arm oCurrent", armMotorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Arm Motor Speed", m_speed);
        // SmartDashboard.putNumber("ArmABS Offset", armAbsEncoder.getPositionOffset());  
        //   Might use global that is set by drive periodic to indicate if driving too fast.

        m_speed = armMotorLeft.getEncoder().getVelocity();

        if (((armAbsEncoder.getAbsolutePosition() < ArmConstants.kMinHeightAbs) && (m_speed < 0)) ||
            ((armAbsEncoder.getAbsolutePosition() > ArmConstants.kMaxHeightAbs) && (m_speed > 0))) {
            armMotorLeft.set(0);
            }

    }

    public void raiseArmAbs(armPositions position){
        if (((armAbsEncoder.getAbsolutePosition() < ArmConstants.kMinHeightAbs) && (position == armPositions.HOME)) ||
            ((armAbsEncoder.getAbsolutePosition() > ArmConstants.kMaxHeightAbs) && (position == armPositions.LVLTRE))) {
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
        armMotorLeft.set(pidOut);
        // TODO: Add a new armPosition that reads a value from the smart dashboard and moves arm to that position.
    }

    public void raiseArm(double speed){
        if (((armAbsEncoder.getAbsolutePosition() < ArmConstants.kOffset) && (speed < 0)) ||
            ((armAbsEncoder.getAbsolutePosition() > ArmConstants.kLVLTRE) && (speed > 0))) {
            armMotorLeft.set(0);
            return;
        }
        armMotorLeft.set(speed);
    }

    public void raiseArm(double raiseSpeed, double lowerSpeed){
        double speed = raiseSpeed - lowerSpeed; //positive output to raise arm
        if (((armAbsEncoder.getAbsolutePosition() < ArmConstants.kOffset) && (speed < 0)) ||
            ((armAbsEncoder.getAbsolutePosition() > ArmConstants.kLVLTRE) && (speed > 0))) {
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

