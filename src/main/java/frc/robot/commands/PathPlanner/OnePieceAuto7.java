package frc.robot.commands.PathPlanner;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.commands.driveCommands.DriveToLevel;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;

public class OnePieceAuto7 extends SequentialCommandGroup {

    public OnePieceAuto7(DriveSubsystem driveSubsystem, 
                        ArmSubsystem armSubsystem,
                        IntakeSubsystem intakeSubsystem
                        ){

        PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Cone Score 7", 
            new PathConstraints(2, 3));
            PathPlannerTrajectory m_secondPath = PathPlanner.loadPath("Cone Reverse 7", 
            new PathConstraints(2, 1));
        PathPlannerTrajectory m_thirdPath = PathPlanner.loadPath("Leave Community 7 (8ft)", 
            new PathConstraints(1, 2));

        
        addCommands(
            new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kIntakePickUp), intakeSubsystem),
            new ArmToPositionWithEnd(armSubsystem, armPositions.LVLTRE).withTimeout(1.6),
            driveSubsystem.followTrajectoryCommand(m_firstPath, true),
            new RunCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kIntakeReverse), intakeSubsystem).withTimeout(.25),
            // new WaitCommand(0.5),
            driveSubsystem.followTrajectoryCommand(m_secondPath, false),
            new InstantCommand(() -> intakeSubsystem.intakeOff()),
            new ParallelCommandGroup(new ArmToPositionWithEnd(armSubsystem, armPositions.HOME).withTimeout(2.0),
            driveSubsystem.followTrajectoryCommand(m_thirdPath, false)),
            new DriveToLevel(driveSubsystem)
            );
    }
}
