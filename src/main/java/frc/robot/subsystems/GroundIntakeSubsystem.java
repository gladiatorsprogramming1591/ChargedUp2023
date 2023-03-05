package frc.robot.subsystems; 

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.CANIDConstants;

public class GroundIntakeSubsystem extends SubsystemBase{
    
    private final TalonSRX RightGroundIntakeMotor = new TalonSRX(CANIDConstants.kRightGroundIntakeCANId);
    private final TalonSRX LeftGroundIntakeMotor = new TalonSRX(CANIDConstants.kLeftGroundIntakeCANId);
    
    private final CANSparkMax GroundIntakeJoint = new CANSparkMax(CANIDConstants.kGroundIntakeJointCANId, MotorType.kBrushless);

    PowerDistribution powerDist = new PowerDistribution(CANIDConstants.kPdhCanId, ModuleType.kRev);
    double intakeCurrent = 2.5;

    double defaultPickUp = GroundIntakeConstants.kIntakePickUp;
    double defaultReverse = GroundIntakeConstants.kIntakeReverse;
    double defaultShoot = GroundIntakeConstants.kIntakeShoot;

    public GroundIntakeSubsystem(){
        RightGroundIntakeMotor.setNeutralMode(NeutralMode.Brake);
        LeftGroundIntakeMotor.setNeutralMode(NeutralMode.Brake);

        RightGroundIntakeMotor.follow(LeftGroundIntakeMotor, FollowerType.PercentOutput);
        RightGroundIntakeMotor.setInverted(true); //TODO (Test): Follower while inverted?

        LeftGroundIntakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, intakeCurrent, intakeCurrent + 1.5, 0.3));
        GroundIntakeJoint.setSmartCurrentLimit(GroundIntakeConstants.GROUND_JOINT_CURRENT_LIMIT_A);

        SmartDashboard.putNumber("GroundIntakePickUp", defaultPickUp);
        SmartDashboard.putNumber("GroundIntakeReverse", defaultReverse);
        SmartDashboard.putNumber("GroundIntakeShoot", defaultShoot);
    }

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

    public void groundJointSpeed(double speed){ // TODO: Eventually needs a PID
        GroundIntakeJoint.set(speed); //TODO: Add soft limits
    }
}

