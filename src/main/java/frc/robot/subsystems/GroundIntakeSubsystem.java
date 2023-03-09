package frc.robot.subsystems; 

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

    private final PIDController GroundIntakePID = new PIDController(GroundIntakeConstants.kp, GroundIntakeConstants.ki, GroundIntakeConstants.kd);

    PowerDistribution powerDist = new PowerDistribution(CANIDConstants.kPdhCanId, ModuleType.kRev);
    double intakeCurrent = 3.0;

    double defaultPickUp = GroundIntakeConstants.kIntakePickUp;
    double defaultReverse = GroundIntakeConstants.kIntakeReverse;
    double defaultShoot = GroundIntakeConstants.kIntakeShoot;

    boolean atPosition = true;
    int positionCheckCount = 0;

    public GroundIntakeSubsystem(){
        RightGroundIntakeMotor.setNeutralMode(NeutralMode.Brake);
        LeftGroundIntakeMotor.setNeutralMode(NeutralMode.Brake);

        RightGroundIntakeMotor.follow(LeftGroundIntakeMotor, FollowerType.PercentOutput);
        RightGroundIntakeMotor.setInverted(true); //TODO (Test): Follower while inverted?

        LeftGroundIntakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, intakeCurrent, intakeCurrent + 1.75, 0.6));
        RightGroundIntakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, intakeCurrent, intakeCurrent + 1.75, 0.6));
        GroundIntakeJoint.setSmartCurrentLimit(GroundIntakeConstants.GROUND_JOINT_CURRENT_LIMIT_A);

        SmartDashboard.putNumber("GroundIntakePickUp", defaultPickUp);
        SmartDashboard.putNumber("GroundIntakeReverse", defaultReverse);
        SmartDashboard.putNumber("GroundIntakeShoot", defaultShoot);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("GroundIntakeJoint oCurrent", GroundIntakeJoint.getOutputCurrent());
        SmartDashboard.putNumber("LeftGroundIntake oCurrent", LeftGroundIntakeMotor.getStatorCurrent());
        SmartDashboard.putNumber("RightGroundIntake oCurrent", RightGroundIntakeMotor.getStatorCurrent());
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

    public void groundJointPosition(double position){
        atPosition = false;
        double speed = MathUtil.clamp( GroundIntakePID.calculate(GroundIntakeJoint.getEncoder().getPosition(), position), 
            GroundIntakeConstants.kMaxReverseOutput, GroundIntakeConstants.kMaxForwardOutput);
        if (GroundIntakeJoint.getEncoder().getVelocity() == 0){
            ++positionCheckCount;
            if (positionCheckCount > 5){
                GroundIntakeJoint.set(0);
                positionCheckCount = 0;
                atPosition = true;
            }
        } else {
            GroundIntakeJoint.set(speed);
            positionCheckCount = 0;
        }
    }

    public boolean groundJointAtPosition(){
        return atPosition;
    }
}

