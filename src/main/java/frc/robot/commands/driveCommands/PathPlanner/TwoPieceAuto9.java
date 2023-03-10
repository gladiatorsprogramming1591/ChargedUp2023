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
// import frc.robot.commands.driveCommands.DriveToLevel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;

public class TwoPieceAuto9 extends SequentialCommandGroup {

    public TwoPieceAuto9(DriveSubsystem driveSubsystem, 
                        ArmSubsystem armSubsystem,
                        IntakeSubsystem intakeSubsystem
                        ){

        PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Cone Score 9", 
            new PathConstraints(2, 3));
            PathPlannerTrajectory m_secondPath = PathPlanner.loadPath("Cone Reverse 9", 
            new PathConstraints(2, 1));
        PathPlannerTrajectory m_thirdPath = PathPlanner.loadPath("Full Drive to Cube 9", 
            new PathConstraints(2, 2));
        // PathPlannerTrajectory m_forthPath = PathPlanner.loadPath("Drive Over Cable", 
        //     new PathConstraints(1, 2));
        // PathPlannerTrajectory m_fifthPath = PathPlanner.loadPath("New Drive to Cube 9", 
        //     new PathConstraints(2, 3));
        // PathPlannerTrajectory m_sixthPath = PathPlanner.loadPath("Go to 8 with Cube", 
        //     new PathConstraints(1, 2));

        
        addCommands(
            new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kIntakePickUp), intakeSubsystem),
            new ArmToPositionWithEnd(armSubsystem, armPositions.LVLTRE).withTimeout(1.6),
            driveSubsystem.followTrajectoryCommand(m_firstPath, true),
            new RunCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kIntakeReverse), intakeSubsystem).withTimeout(.25),
            // new WaitCommand(0.5),
            driveSubsystem.followTrajectoryCommand(m_secondPath, false),
            new InstantCommand(() -> intakeSubsystem.intakeOff()),
            new ParallelCommandGroup(new ArmToPositionWithEnd(armSubsystem, armPositions.HOME).withTimeout(2.0),
                driveSubsystem.followTrajectoryCommand(m_thirdPath, false))
            // driveSubsystem.followTrajectoryCommand(m_forthPath, false),
            // driveSubsystem.followTrajectoryCommand(m_fifthPath, false)
            // new DriveToLevel(driveSubsystem)
            );
    }
}
