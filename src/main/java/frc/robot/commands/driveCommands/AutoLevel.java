package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevel extends SequentialCommandGroup {

    public AutoLevel(DriveSubsystem driveSubsystem) {
        addCommands(
        new DriveToAngle(driveSubsystem),
        new DriveToLevel(driveSubsystem)
        );
    }
}
