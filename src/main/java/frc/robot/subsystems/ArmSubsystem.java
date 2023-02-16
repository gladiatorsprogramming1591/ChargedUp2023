package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import java.util.EnumMap;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{
    public enum armPositions{
        LVLONE, 
        LVLTWO, 
        LVLTRE,
        HOME;
    }
        
    private final CANSparkMax armMotorLeft = new CANSparkMax(Constants.CANIDConstants.kLeftArmCANId, MotorType.kBrushless); 
    private final CANSparkMax armMotorRight = new CANSparkMax(Constants.CANIDConstants.kRightArmCANId, MotorType.kBrushless); 
    private final SparkMaxPIDController armPID = armMotorLeft.getPIDController(); 
    // private final ProfiledPIDController armPID = 
    //     new ProfiledPIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD, 
    //     new Constraints(ArmConstants.kArmMaxVel, ArmConstants.kArmMaxAcc));
    private final RelativeEncoder armEncoder = armMotorLeft.getEncoder(); 
    private double baseEncoderPosition = 0; 

    EnumMap<armPositions, Double> map = new EnumMap<>(armPositions.class); 

    public ArmSubsystem(){
        armMotorLeft.setInverted(true);
        armMotorRight.follow(armMotorLeft, true); 
 
        // Adding elements to the Map
        // using standard put() method
        map.put(armPositions.LVLONE, 20.0);
        map.put(armPositions.LVLTWO, 40.0);
        map.put(armPositions.LVLTRE, 72.0);
        map.put(armPositions.HOME, 0.0); //TODO (requires bot): empirically measure encoder positions and update here

        armPID.setP(ArmConstants.kArmP);
        armPID.setI(ArmConstants.kArmI);
        armPID.setD(ArmConstants.kArmD);
        armPID.setFF(ArmConstants.kArmFF);
        armPID.setOutputRange(ArmConstants.kArmMinOutput, ArmConstants.kArmMaxOutput);

        baseEncoderPosition = armEncoder.getPosition();

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
    int smartMotionSlot = 0;
    armPID.setSmartMotionMaxVelocity(ArmConstants.kArmMaxVel, smartMotionSlot);
    armPID.setSmartMotionMinOutputVelocity(ArmConstants.kArmMinVel, smartMotionSlot);
    armPID.setSmartMotionMaxAccel(ArmConstants.kArmMaxAcc, smartMotionSlot);
    armPID.setSmartMotionAllowedClosedLoopError(ArmConstants.kAllowedErr, smartMotionSlot);

    SmartDashboard.putNumber("Arm base position",baseEncoderPosition);
    SmartDashboard.putNumber("Arm Enc", armMotorLeft.getEncoder().getPosition());

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Enc", armMotorLeft.getEncoder().getPosition());
        // TODO: Low Priority: If driving too fast, lower arm to home. 
        //   Might use global that is set by drive periodic to indicate if driving too fast.
    }

    public void raiseArm(armPositions position){
        double ref = map.get(position);
        SmartDashboard.putNumber("Arm Target Pos", ref);
        armPID.setReference(ref, CANSparkMax.ControlType.kPosition);
        armPID.setFeedbackDevice(armEncoder);
        // TODO: Add a new armPosition that reads a value from teh smart dashboard and moves arm to that position.
    }

    public void raiseArm(double speed){
        armMotorLeft.set(speed);
    }

    public void raiseArm(double raiseSpeed, double lowerSpeed){
        double speed = raiseSpeed - lowerSpeed; //positive output to raise arm
        armMotorLeft.set(speed);
    }

    public boolean atLevel(armPositions pos){
        double currentEncoderPosition = armEncoder.getPosition();
        return (Math.abs(currentEncoderPosition - map.get(pos)) < Constants.ArmConstants.kAllowedErr);
    }

}

