package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

enum armPositions{
    LVLONE, 
    LVLTWO, 
    LVLTRE; 

}

class ArmSubsystem extends SubsystemBase{
    private final CANSparkMax armMotor = new CANSparkMax(Constants.DriveConstants.kArmCANId, null); 
    private final SparkMaxPIDController armPID = armMotor.getPIDController(); 
    // private final RelativeEncoder armEncoder = ; 

    public ArmSubsystem(){
        


    }

    public void raiseArm(armPositions position){
        switch(position){
            case LVLONE:
            armPID.setReference(100, ControlType.kPosition); //100 needs to be updated to accurate number
            //set the target position for our PID controller 
            case LVLTWO: 
            case LVLTRE: 

        }

    }

}
