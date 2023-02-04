package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    LVLTRE; 

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
        map.put(armPositions.LVLTRE, 30); //TODO: empirically measure encoder positions and update here

        armPID.setP(ArmConstants.kArmP);
        armPID.setI(ArmConstants.kArmI);
        armPID.setD(ArmConstants.kArmD);
        armPID.setFF(ArmConstants.kArmFF);
        armPID.setOutputRange(ArmConstants.kArmMinOutput, ArmConstants.kArmMaxOutput);

        baseEncoderPosition = armEncoder.getPosition(); 

        SmartDashboard.putNumber("Arm base position: ", baseEncoderPosition); 

    }

    public void raiseArm(armPositions position){
        armPID.setReference(map.get(position), ControlType.kPosition);
        armPID.setFeedbackDevice(armEncoder);

        }

    public void raiseArm(double speed){
        armMotorLeft.set(speed);

    }

}

