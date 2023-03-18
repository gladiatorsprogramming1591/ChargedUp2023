package frc.robot.commands.PathPlanner;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.commands.driveCommands.DriveToLevel;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;

public class C5OneCubeLevel extends SequentialCommandGroup {

    public C5OneCubeLevel(DriveSubsystem driveSubsystem, 
                        ArmSubsystem armSubsystem,
                        IntakeSubsystem intakeSubsystem
                        ){

        PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Cone Score 5", 
            new PathConstraints(2, 3));
        PathPlannerTrajectory m_secondPath = PathPlanner.loadPath("Cone Reverse 5", 
            new PathConstraints(2, 1));       
        PathPlannerTrajectory m_thirdPath = PathPlanner.loadPath("Leave 5 Level", 
            new PathConstraints(1.4, 3));
        
        addCommands(
            new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kIntakeReverse), intakeSubsystem),
            new ArmToPositionWithEnd(armSubsystem, armPositions.LVLTRE).withTimeout(1.55),
            driveSubsystem.followTrajectoryCommand(m_firstPath, true),
            new RunCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kIntakePickUp + 0.2), intakeSubsystem).withTimeout(.2),
            // new WaitCommand(0.5),
            driveSubsystem.followTrajectoryCommand(m_secondPath, false),
            new InstantCommand(() -> intakeSubsystem.intakeOff()),
            new ArmToPositionWithEnd(armSubsystem, armPositions.HOME).withTimeout(1.25),
            driveSubsystem.followTrajectoryCommand(m_thirdPath, false),
            new DriveToLevel(driveSubsystem)
            );
    }
}
