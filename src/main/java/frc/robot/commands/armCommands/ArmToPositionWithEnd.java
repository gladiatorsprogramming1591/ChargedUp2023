package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPositionWithEnd extends CommandBase {
    private ArmSubsystem m_arm;
    private ArmSubsystem.armPositions m_targetPos;

    public ArmToPositionWithEnd(ArmSubsystem arm, ArmSubsystem.armPositions pos){
        m_arm = arm;
        m_targetPos = pos;
        addRequirements(m_arm);
    }

    @Override
    public void execute(){
        m_arm.raiseArmAbs(m_targetPos);
    }

    @Override
    public boolean isFinished(){
        return m_arm.atPosition(m_targetPos);
    }

    @Override
    public void end(boolean isInterrupted){
        m_arm.raiseArm(0); // TODO: Do we want this to only stop the arm motor if not interrupted?
    }
}
