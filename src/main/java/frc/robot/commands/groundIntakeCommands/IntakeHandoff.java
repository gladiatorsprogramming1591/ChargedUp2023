package frc.robot.commands.groundIntakeCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem.GroundIntake;
import frc.robot.subsystems.GroundIntakeSubsystem.GroundJoint;

public class IntakeHandoff extends SequentialCommandGroup {
    private GroundIntake m_groundIntake;
    private GroundJoint m_groundJoint;
    private IntakeSubsystem m_mainIntake;

    public IntakeHandoff(GroundIntake GIntake, GroundJoint GJoint, IntakeSubsystem MIntake){
        m_groundIntake = GIntake;
        m_mainIntake = MIntake;
        m_groundJoint = GJoint;
        addRequirements(m_mainIntake, m_groundIntake, m_groundJoint);
        addCommands(
            new RunCommand(() -> m_groundJoint.groundJointPosition(Constants.GroundIntakeConstants.kInPosition))
                .alongWith( new RunCommand(
                () -> m_groundIntake.groundIntakeSpeed(
                GroundIntakeConstants.kDefaultSpeed + 0.2))).withTimeout(2.5),

            new RunCommand(() -> m_mainIntake.intakeOn(IntakeConstants.kIntakeReverse))
                .alongWith(new RunCommand(() -> m_groundIntake.groundIntakeReverse())).withTimeout(1.0)
        
        );
    }

}
