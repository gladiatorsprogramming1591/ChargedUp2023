package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevel extends SequentialCommandGroup {

    public AutoLevel(DriveSubsystem driveSubsystem) {
        addCommands(
        // new DriveToAngle(driveSubsystem), // TODO (fix): Currently does not end
        driveSubsystem.DriveCommand(0.2).withTimeout(1.5),
        new DriveToLevel(driveSubsystem)
        );
    }
}

