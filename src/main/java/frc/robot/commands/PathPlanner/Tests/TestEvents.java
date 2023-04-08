package frc.robot.commands.PathPlanner.Tests;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;

public class TestEvents extends SequentialCommandGroup {

    // TODO: Path group name as parameter
    public TestEvents(DriveSubsystem driveSubsystem, 
                        ArmSubsystem armSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LEDs LEDs
                        ){

        List<PathPlannerTrajectory> m_path = PathPlanner.loadPathGroup("Test Events",
        // PathPlannerTrajectory m_path = PathPlanner.loadPath("Test Events",
            // Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            // Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
            1.0, 1.0);
            
        // TODO: Create loop instead of 4 seperate commands (To pass in different paths in parameter)
        // m_path.forEach(addCommands(new FollowPathWithEvents(
        //     driveSubsystem.followTrajectoryCommand(m_path.get(0), true),
        //     m_path.get(0).getMarkers(),
        //     Constants.AutoConstants.AUTO_EVENT_MAP)));

        addCommands(
                new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kConePickUp), intakeSubsystem),
                new ArmToPositionWithEnd(armSubsystem, armPositions.LVLTRE).withTimeout(1.6),
                new FollowPathWithEvents(
                    driveSubsystem.followTrajectoryCommand(m_path.get(0), true),
                    m_path.get(0).getMarkers(),
                    Constants.AutoConstants.AUTO_EVENT_MAP),
                new InstantCommand(() -> LEDs.setColor(LEDs.WHITE)),
                new WaitCommand(2.0),
                new FollowPathWithEvents(
                    driveSubsystem.followTrajectoryCommand(m_path.get(1), false),
                    m_path.get(1).getMarkers(),
                    Constants.AutoConstants.AUTO_EVENT_MAP),
                new InstantCommand(() -> LEDs.setColor(LEDs.TEAL)),
                new WaitCommand(2.0),
                new FollowPathWithEvents(
                    driveSubsystem.followTrajectoryCommand(m_path.get(2), false),
                    m_path.get(2).getMarkers(),
                    Constants.AutoConstants.AUTO_EVENT_MAP),
                new InstantCommand(() -> LEDs.setColor(LEDs.BLUE)),
                new RunCommand(() -> LEDs.cycle(), LEDs).withTimeout(2.0)


                // new FollowPathWithEvents(
                //     driveSubsystem.followTrajectoryCommand(m_path.get(3), false),
                //     m_path.get(3).getMarkers(),
                //     Constants.AutoConstants.AUTO_EVENT_MAP)
            );
    }
}
