package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevel extends SequentialCommandGroup {

    public AutoLevel(DriveSubsystem m_DriveSubsystem) {
        addCommands(
        new DriveToAngle(m_DriveSubsystem),
        new DriveToLevel(m_DriveSubsystem)
        );
    }
}
