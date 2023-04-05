package frc.robot.commands.groundIntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.GroundArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.GroundIntakeSubsystem.GroundIntake;
import frc.robot.subsystems.GroundIntakeSubsystem.GroundJoint;
import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;

public class IntakeShoot extends SequentialCommandGroup {
    private GroundIntake m_groundIntake;
    private GroundJoint m_groundJoint;
    private IntakeSubsystem m_mainIntake;

    public IntakeShoot(GroundIntake GIntake, GroundJoint GJoint){
        m_groundIntake = GIntake;
        m_groundJoint = GJoint;
        addRequirements(m_mainIntake, m_groundIntake, m_groundJoint);
        addCommands(
            new InstantCommand(() -> m_groundJoint.setAtPositionBoolean(false)),
            new RunCommand(() -> m_groundJoint.groundJointPosition(Constants.GroundArmConstants.kInPosition))
                .until(() -> m_groundJoint.groundJointAtPosition()).withTimeout(1.5)
                .raceWith( new RunCommand( () -> m_groundIntake.groundIntakeSpeed(GroundArmConstants.kDefaultSpeed + 0.15))),
                // .withTimeout(2.5),

            new RunCommand(() -> m_mainIntake.intakeOn(IntakeConstants.kConeEject))
                .alongWith( new RunCommand(() -> m_groundIntake.groundIntakeReverse())).withTimeout(0.75)
        
        );
    }

}
