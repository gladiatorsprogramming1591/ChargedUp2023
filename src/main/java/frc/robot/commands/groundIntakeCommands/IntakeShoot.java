package frc.robot.commands.groundIntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.GroundArmConstants;
import frc.robot.subsystems.GroundIntakeSubsystem.GroundIntake;
import frc.robot.subsystems.GroundIntakeSubsystem.GroundJoint;

public class IntakeShoot extends SequentialCommandGroup {
    private GroundIntake m_groundIntake;
    private GroundJoint m_groundJoint;

    public IntakeShoot(GroundIntake GIntake, GroundJoint GJoint){
        m_groundIntake = GIntake;
        m_groundJoint = GJoint;
        addRequirements(m_groundIntake, m_groundJoint);
        addCommands(
            new InstantCommand(() -> m_groundJoint.setAtPositionBoolean(false)),
            new RunCommand(() -> m_groundJoint.groundJointPosition(Constants.GroundArmConstants.kShootPosition))
                .until(() -> m_groundJoint.groundJointAtPosition()).withTimeout(1.5)    // REVIEW 'atPosition' for 'kShootPosition' before testing
                .raceWith( new RunCommand( () -> m_groundIntake.groundIntakeSpeed(GroundArmConstants.kDefaultSpeed + 0.15))),

                new RunCommand(() -> m_groundIntake.groundIntakeShoot(), m_groundIntake).withTimeout(0.75)
        );
    }

}
