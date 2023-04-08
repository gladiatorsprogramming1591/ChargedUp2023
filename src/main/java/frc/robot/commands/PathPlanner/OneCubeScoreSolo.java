package frc.robot.commands.PathPlanner;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;

public class OneCubeScoreSolo extends SequentialCommandGroup {

    public OneCubeScoreSolo(DriveSubsystem driveSubsystem, 
                        ArmSubsystem armSubsystem,
                        IntakeSubsystem intakeSubsystem
                        ){

        PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Cube Score 5", 
            new PathConstraints(2, 3));
            PathPlannerTrajectory m_secondPath = PathPlanner.loadPath("Cube Reverse 5", 
            new PathConstraints(2, 1));

        
        addCommands(
            new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kConePickUp), intakeSubsystem),
            new ArmToPositionWithEnd(armSubsystem, armPositions.LVLTRE).withTimeout(1.6),
            driveSubsystem.followTrajectoryCommand(m_firstPath, true),
            new RunCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kConeEject), intakeSubsystem).withTimeout(.25),
            driveSubsystem.followTrajectoryCommand(m_secondPath, false),
            new InstantCommand(() -> intakeSubsystem.intakeOff()),
            new ArmToPositionWithEnd(armSubsystem, armPositions.HOME).withTimeout(2.0));
    }
}
