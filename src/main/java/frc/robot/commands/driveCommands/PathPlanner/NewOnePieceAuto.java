package frc.robot.commands.driveCommands.PathPlanner;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class NewOnePieceAuto extends SequentialCommandGroup {

    // TODO: Path group name as parameter
    public NewOnePieceAuto(DriveSubsystem driveSubsystem, 
                        ArmSubsystem armSubsystem,
                        IntakeSubsystem intakeSubsystem
                        ){

        List<PathPlannerTrajectory> m_path = PathPlanner.loadPathGroup("One Piece Auto",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

        // TODO: Create loop instead of 4 seperate commands
        // m_path.forEach(addCommands(new FollowPathWithEvents(
        //     driveSubsystem.followTrajectoryCommand(m_path.get(0), true),
        //     m_path.get(0).getMarkers(),
        //     Constants.AutoConstants.AUTO_EVENT_MAP)));

        addCommands(
                new FollowPathWithEvents(
                    driveSubsystem.followTrajectoryCommand(m_path.get(0), true),
                    m_path.get(0).getMarkers(),
                    Constants.AutoConstants.AUTO_EVENT_MAP),
                new FollowPathWithEvents(
                    driveSubsystem.followTrajectoryCommand(m_path.get(1), false),
                    m_path.get(1).getMarkers(),
                    Constants.AutoConstants.AUTO_EVENT_MAP),
                new FollowPathWithEvents(
                    driveSubsystem.followTrajectoryCommand(m_path.get(2), false),
                    m_path.get(2).getMarkers(),
                    Constants.AutoConstants.AUTO_EVENT_MAP),
                new FollowPathWithEvents(
                    driveSubsystem.followTrajectoryCommand(m_path.get(3), false),
                    m_path.get(3).getMarkers(),
                    Constants.AutoConstants.AUTO_EVENT_MAP)
            );
    }
}
