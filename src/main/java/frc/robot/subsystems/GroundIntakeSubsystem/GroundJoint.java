package frc.robot.subsystems.GroundIntakeSubsystem; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.CANIDConstants;

public class GroundJoint extends SubsystemBase{
        
    private final CANSparkMax GroundIntakeJoint = new CANSparkMax(CANIDConstants.kGroundIntakeJointCANId, MotorType.kBrushless);

    private final PIDController GroundIntakePID = new PIDController(GroundIntakeConstants.kp, GroundIntakeConstants.ki, GroundIntakeConstants.kd);

    boolean atPosition;
    int positionCheckCount = 0;

    public GroundJoint(){
        GroundIntakeJoint.restoreFactoryDefaults();
        GroundIntakeJoint.setIdleMode(IdleMode.kBrake);

        GroundIntakeJoint.setSmartCurrentLimit(GroundIntakeConstants.GROUND_JOINT_CURRENT_LIMIT_A);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("GroundIntakeJoint oCurrent", GroundIntakeJoint.getOutputCurrent());
        SmartDashboard.putNumber("GroundIntakeJoint Position", GroundIntakeJoint.getEncoder().getPosition());
    }

    // Joint Speed: Positive  is up, negative speed down

    public void groundJointSpeed(double speed){ // TODO: Eventually needs a PID
        GroundIntakeJoint.set(speed); //TODO: Add soft limits
    }    

    public void groundJointPosition(double position){
        atPosition = false;
        double speed = MathUtil.applyDeadband(MathUtil.clamp(GroundIntakePID.calculate(GroundIntakeJoint.getEncoder().getPosition(), position), 
            GroundIntakeConstants.kMaxJointOutSpeed, GroundIntakeConstants.kMaxJointInSpeed),
            GroundIntakeConstants.kPIDDeadband);
        SmartDashboard.putNumber("Ground Intake PidOut Speed", speed);

        // between -0.1 and 0.1 RPM for 100ms to set atPosition = true
        // if (GroundIntakeJoint.getEncoder().getVelocity() == 0){
        // if (GroundIntakeJoint.getEncoder().getVelocity() <= GroundIntakeConstants.kOffVelocity && 
        //     GroundIntakeJoint.getEncoder().getVelocity() >= -GroundIntakeConstants.kOffVelocity){
        //     ++positionCheckCount;

        //     if (positionCheckCount > 100){
        //         GroundIntakeJoint.set(0);
        //         positionCheckCount = 0;
        //         atPosition = true;
        //     }

        // } else {
            GroundIntakeJoint.set(speed);
            positionCheckCount = 0;
        // }
    }

    public boolean groundJointAtPosition(){
        // return atPosition;
        return false;
    }

    public void zeroEncoder(){
        GroundIntakeJoint.getEncoder().setPosition(0.0);
    }
}

