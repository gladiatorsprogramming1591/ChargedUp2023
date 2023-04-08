package frc.robot.subsystems.MainIntakeSubsystem; 

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;

public class IntakeSubsystem extends SubsystemBase{
    
    private final TalonSRX intakeMotor = new TalonSRX(Constants.CANIDConstants.kIntakeCANId);
    
    // TODO: Is powerDist needed?
    PowerDistribution powerDist = new PowerDistribution(Constants.CANIDConstants.kPdhCanId, ModuleType.kRev);
    double intakeCurrent = 3.75; //2.75

    public IntakeSubsystem(){
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 
            intakeCurrent, intakeCurrent + 2.75, 0.6)); // +1.75, 0.6 seconds
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake oCurrent", intakeMotor.getStatorCurrent());
        SmartDashboard.putNumber("Intake iCurrent", intakeMotor.getSupplyCurrent());
    }

    public void intakeOn(double speed){
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void intakeOff(){
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void intakeStall(double joystick, double speed){
        if (joystick > 0) intakeMotor.set(ControlMode.PercentOutput, speed);
        else if (joystick < 0) intakeMotor.set(ControlMode.PercentOutput, -speed);
        else intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public void operatorReverse(double manipulatorY, double speed){
        // Joystick deadband to handle nonzero reading of stick while not used
        if (manipulatorY > OIConstants.kJoystickDeadband) intakeMotor.set(ControlMode.PercentOutput, speed);
        else if (manipulatorY < -OIConstants.kJoystickDeadband) intakeMotor.set(ControlMode.PercentOutput, -speed);
        else intakeMotor.set(ControlMode.PercentOutput, 0);
    }
}

