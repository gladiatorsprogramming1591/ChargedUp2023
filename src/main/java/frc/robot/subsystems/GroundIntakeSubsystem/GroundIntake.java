package frc.robot.subsystems.GroundIntakeSubsystem; 

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundArmConstants;
import frc.robot.Constants.CANIDConstants;

public class GroundIntake extends SubsystemBase{
    
    private final TalonSRX RightGroundIntakeMotor = new TalonSRX(CANIDConstants.kRightGroundIntakeCANId);
    private final TalonSRX LeftGroundIntakeMotor = new TalonSRX(CANIDConstants.kLeftGroundIntakeCANId);

    double intakeCurrent = 3.0;

    double defaultPickUp = GroundArmConstants.kIntakePickUp;
    double defaultReverse = GroundArmConstants.kIntakeReverse;
    double defaultShoot = GroundArmConstants.kIntakeShoot;

    boolean atPosition;
    int positionCheckCount = 0;

    public GroundIntake(){
        RightGroundIntakeMotor.configFactoryDefault();
        LeftGroundIntakeMotor.configFactoryDefault();
        RightGroundIntakeMotor.setNeutralMode(NeutralMode.Brake);
        LeftGroundIntakeMotor.setNeutralMode(NeutralMode.Brake);

        RightGroundIntakeMotor.follow(LeftGroundIntakeMotor, FollowerType.PercentOutput);
        RightGroundIntakeMotor.setInverted(true);

        LeftGroundIntakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, intakeCurrent, intakeCurrent + 1.75, 0.1));
        RightGroundIntakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, intakeCurrent, intakeCurrent + 1.75, 0.1));

        SmartDashboard.putNumber("GroundIntakePickUp", defaultPickUp);
        SmartDashboard.putNumber("GroundIntakeReverse", defaultReverse);
        SmartDashboard.putNumber("GroundIntakeShoot", defaultShoot);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LeftGroundIntake oCurrent", LeftGroundIntakeMotor.getStatorCurrent());
        SmartDashboard.putNumber("RightGroundIntake oCurrent", RightGroundIntakeMotor.getStatorCurrent());
        SmartDashboard.putNumber("LeftGroundIntake iCurrent", LeftGroundIntakeMotor.getSupplyCurrent());
        SmartDashboard.putNumber("RightGroundIntake iCurrent", RightGroundIntakeMotor.getSupplyCurrent());
    }

    // Joint Speed: Positive  is up, negative speed down
    // Intake Speed: Positive is pick-up, negative is eject

    public void groundIntakePickUp(){
        double speed = SmartDashboard.getNumber("GroundIntakePickUp", defaultPickUp);
        LeftGroundIntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void groundIntakeReverse(){
        double speed = SmartDashboard.getNumber("GroundIntakeReverse", defaultReverse);
        LeftGroundIntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void groundIntakeShoot(){
        double speed = SmartDashboard.getNumber("GroundIntakeShoot", defaultShoot);
        LeftGroundIntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void groundIntakeOff(){
        LeftGroundIntakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void groundIntakeSpeed(double speed){
        LeftGroundIntakeMotor.set(ControlMode.PercentOutput, speed);
    }

}

