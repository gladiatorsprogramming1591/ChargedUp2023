package frc.robot.subsystems; 

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import java.util.EnumMap;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem.armPositions;

public class IntakeSubsystem extends SubsystemBase{
        
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.DriveConstants.kLeftArmCANId, MotorType.kBrushed); 

    PowerDistribution powerDist = new PowerDistribution(Constants.kPdhCanId, ModuleType.kRev);
    double intakeCurrent = 0;

    public IntakeSubsystem(){
        // TODO: Determine if smart current limit can be used with brushed motors with no encoders
        // intakeMotor.setSmartCurrentLimit(Constants.INTAKE_CURRENT_LIMIT_A);

    }

    public void intakeOn(double speed){
        // Try monitoring the current limit from PDH
        intakeCurrent = powerDist.getCurrent(Constants.IntakeConstants.kPdhChannel);
        SmartDashboard.putNumber("Intake Current",intakeCurrent);

        if (intakeCurrent < Constants.INTAKE_CURRENT_LIMIT_A){
            intakeMotor.set(speed);
        } else {
            intakeMotor.set(Math.copySign(Constants.IntakeConstants.kStallSpeed, speed));
        }
    }

    public void intakeOff(double speed){
        intakeMotor.set(0);
    }
}

