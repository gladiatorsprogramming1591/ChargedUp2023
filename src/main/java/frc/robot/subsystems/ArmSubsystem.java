package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import java.util.EnumMap;
import frc.robot.Constants.ArmConstants;

enum armPositions{
    LVLONE, 
    LVLTWO, 
    LVLTRE,
    HOME;
}

public class ArmSubsystem extends SubsystemBase{

    private final CANSparkMax armMotorLeft = new CANSparkMax(Constants.DriveConstants.kLeftArmCANId, MotorType.kBrushless); 
    private final CANSparkMax armMotorRight = new CANSparkMax(Constants.DriveConstants.kRightArmCANId, MotorType.kBrushless); 
    private final SparkMaxPIDController armPID = armMotorLeft.getPIDController(); 
    private final RelativeEncoder armEncoder = armMotorLeft.getEncoder(); 
    private double baseEncoderPosition = 0; 

    EnumMap<armPositions, Integer> map = new EnumMap<>(armPositions.class); 

    public ArmSubsystem(){
        armMotorRight.follow(armMotorLeft, true); 

        //TODO: set up current limits
 
        // Adding elements to the Map
        // using standard put() method
        map.put(armPositions.LVLONE, 10);
        map.put(armPositions.LVLTWO, 20);
        map.put(armPositions.LVLTRE, 30);
        map.put(armPositions.HOME, 0); //TODO: empirically measure encoder positions and update here

        armPID.setP(ArmConstants.kArmP);
        armPID.setI(ArmConstants.kArmI);
        armPID.setD(ArmConstants.kArmD);
        armPID.setFF(ArmConstants.kArmFF);
        armPID.setOutputRange(ArmConstants.kArmMinOutput, ArmConstants.kArmMaxOutput);

        baseEncoderPosition = armEncoder.getPosition(); 

        armMotorLeft.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT_A);
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

    public void raiseArm(armPositions position){
        armPID.setReference(map.get(position), ControlType.kPosition);
        armPID.setFeedbackDevice(armEncoder);

        }

    public void raiseArm(double speed){
        armMotorLeft.set(speed);

    }

}

