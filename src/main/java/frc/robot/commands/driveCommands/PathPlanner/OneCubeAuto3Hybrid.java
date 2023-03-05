package frc.robot.commands.driveCommands.PathPlanner;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
// import frc.robot.commands.driveCommands.AutoLevel;
import frc.robot.commands.driveCommands.DriveToLevel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;

public class OneCubeAuto3Hybrid extends SequentialCommandGroup {

    public OneCubeAuto3Hybrid(DriveSubsystem driveSubsystem, 
                        ArmSubsystem armSubsystem,
                        IntakeSubsystem intakeSubsystem
                        ){

        PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Cone Score 3", 
            new PathConstraints(2, 3));
            PathPlannerTrajectory m_secondPath = PathPlanner.loadPath("Cone Reverse 3", 
            new PathConstraints(2, 1));
        PathPlannerTrajectory m_thirdPath = PathPlanner.loadPath("Leave Community 3 (4ft)",
            new PathConstraints(2, 3));

        
        addCommands(
            new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kIntakeReverse), intakeSubsystem),
            new ArmToPositionWithEnd(armSubsystem, armPositions.LVLONE).withTimeout(1.2),
            new RunCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kIntakePickUp + 0.2), intakeSubsystem).withTimeout(.25),
            new ParallelCommandGroup(new ArmToPositionWithEnd(armSubsystem, armPositions.HOME).withTimeout(0.8),
            driveSubsystem.followTrajectoryCommand(m_firstPath, true)),
            // new WaitCommand(0.5),
            new InstantCommand(() -> intakeSubsystem.intakeOff()),
            driveSubsystem.followTrajectoryCommand(m_secondPath, false),
            driveSubsystem.followTrajectoryCommand(m_thirdPath, false),
            // new AutoLevel(driveSubsystem)
            new DriveToLevel(driveSubsystem)
            );
    }
}
