package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
        
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.CANIDConstants.kLeftArmCANId, MotorType.kBrushed); 

    PowerDistribution powerDist = new PowerDistribution(Constants.CANIDConstants.kPdhCanId, ModuleType.kRev);
    double intakeCurrent = 0;

    public IntakeSubsystem(){

    }
    // TODO: Add Sensors (1 Color and 2 Distance[for cone and cube])
    // Color sensor to set motor direction, Distance to determine alignment to grid
    public void intakeOn(double speed){
        // TODO: Try monitoring the current limit from PDH
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

