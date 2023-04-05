package frc.robot.subsystems.GroundIntakeSubsystem; 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundArmConstants;
import frc.robot.Constants.CANIDConstants;

public class GroundJoint extends SubsystemBase{
        
    private final CANSparkMax GroundIntakeJoint = new CANSparkMax(CANIDConstants.kGroundIntakeJointCANId, MotorType.kBrushless);

    private final PIDController GroundIntakePID = new PIDController(GroundArmConstants.kOutP, GroundArmConstants.ki, GroundArmConstants.kd);

    double m_encoderPos;
    boolean atPosition;
    double currentPosition;

    public GroundJoint(){
        GroundIntakeJoint.restoreFactoryDefaults();
        GroundIntakeJoint.setIdleMode(IdleMode.kBrake);

        GroundIntakeJoint.setSmartCurrentLimit(GroundArmConstants.GROUND_JOINT_CURRENT_LIMIT_A);
    }

    @Override
    public void periodic() {
        m_encoderPos = GroundIntakeJoint.getEncoder().getPosition();
        SmartDashboard.putNumber("GroundIntakeJoint oCurrent", GroundIntakeJoint.getOutputCurrent());
        SmartDashboard.putNumber("GroundIntakeJoint Position", m_encoderPos);
        SmartDashboard.putBoolean("GroundJoint atPosition", atPosition);
    }

    // Joint Speed: Positive  is up, negative speed down

    public void groundJointSpeed(double speed){

        GroundIntakeJoint.set(speed); //TODO: Add soft limits
    }

    public void groundJointEmergencyControl(double speed){

        GroundIntakeJoint.set(speed);

        this.zeroEncoder();
    }

    public void groundJointOff(){
        GroundIntakeJoint.set(0);
    }

    public void setAtPositionBoolean(boolean atPosRequest){
        atPosition = atPosRequest;
    }

    public void groundJointPosition(double position){
        atPosition = false;
        double speed = MathUtil.applyDeadband(MathUtil.clamp(GroundIntakePID.calculate(m_encoderPos, position), 
            GroundArmConstants.kMaxJointOutSpeed, GroundArmConstants.kMaxJointInSpeed),
            GroundArmConstants.kPIDDeadband);
        SmartDashboard.putNumber("Ground Intake PidOut Speed", speed);
        SmartDashboard.putNumber("Ground Intake PidOut Setpoint", position);

        if (position == GroundArmConstants.kInPosition){
            GroundIntakePID.setP(GroundArmConstants.kInP);
            if (m_encoderPos > (GroundArmConstants.kInPosition - GroundArmConstants.kJointTolerance*10.0)){ // Stops sooner before setpoint to protect "hardstop" (Gearbox)
                atPosition = true;
            }
        } else  {
        if (position == GroundArmConstants.kOutPosition){
            GroundIntakePID.setP(GroundArmConstants.kOutP);
            if (m_encoderPos < (GroundArmConstants.kOutPosition + GroundArmConstants.kJointTolerance)){
                atPosition = true;
            }
        } else {
        if (position == GroundArmConstants.kShootPosition){
            if (GroundIntakeJoint.getEncoder().getPosition() > (GroundArmConstants.kOutPosition / 2)){    // if Starting from inPosition
                GroundIntakePID.setP(GroundArmConstants.kInP);
                if (m_encoderPos < (GroundArmConstants.kShootPosition - GroundArmConstants.kShootJointTolerance)){
                    atPosition = true;
                }    
            } else {
                GroundIntakePID.setP(GroundArmConstants.kOutP);
                if (m_encoderPos < (GroundArmConstants.kShootPosition + GroundArmConstants.kShootJointTolerance)){
                    atPosition = true;
                }  
            }
            }
        }
        }
        GroundIntakeJoint.set(speed);

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
            // GroundIntakeJoint.set(speed);
            // positionCheckCount = 0;
        // }
    }

    public boolean groundJointAtPosition(){
        return atPosition;
    }

    public void zeroEncoder(){
        GroundIntakeJoint.getEncoder().setPosition(0.0);
    }
}

