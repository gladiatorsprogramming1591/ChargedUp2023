package frc.robot.commands.PathPlanner;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.commands.driveCommands.DriveToLevel;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;

public class C1ThreePiece extends SequentialCommandGroup {

    public C1ThreePiece(
        boolean isRED,
        DriveSubsystem driveSubsystem, 
        ArmSubsystem armSubsystem,
        IntakeSubsystem intakeSubsystem,
        LEDs LED
        ){

    PathPlannerTrajectory m_conePath = PathPlanner.loadPath("Cone Score 1", 
        new PathConstraints(2, 3));
    PathPlannerTrajectory m_1stPickUpPath = PathPlanner.loadPath("Cube to shoot 2 from 1 Grid", 
        new PathConstraints(4.0, 3.0));
    PathPlannerTrajectory m_2ndPickUpPath = PathPlanner.loadPath("Cube to shoot 2 from 2 Grid bot45", 
        new PathConstraints(4.0, 3.0));
    PathPlannerTrajectory m_balancePath;

    if (isRED)
    m_balancePath = PathPlanner.loadPath("Balance from 2 BLUE 3Piece", 
        new PathConstraints(3.0, 4.0));
    else
    m_balancePath = PathPlanner.loadPath("Balance from 2 BLUE 3Piece",   // Add Red Option if Charge Station behaves differently
        new PathConstraints(3.0, 4.0));
    
    addCommands(
        new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kConePickUp), intakeSubsystem),
        new ArmToPositionWithEnd(armSubsystem, armPositions.LVLTRE).withTimeout(1.20),
        new FollowPathWithEvents(
            driveSubsystem.followTrajectoryCommand(m_conePath, true),
            m_conePath.getMarkers(),
            Constants.AutoConstants.AUTO_EVENT_MAP),
        new RunCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kConeEject), intakeSubsystem).withTimeout(.25), // TODO: Possible to Reduce?

        new FollowPathWithEvents(
            driveSubsystem.followTrajectoryCommand(m_1stPickUpPath, false),
            m_1stPickUpPath.getMarkers(),
            Constants.AutoConstants.AUTO_EVENT_MAP),

        new InstantCommand(() -> LED.setColor(LED.BLUE)),
        new FollowPathWithEvents(
            driveSubsystem.followTrajectoryCommand(m_2ndPickUpPath, false),
            m_2ndPickUpPath.getMarkers(),
            Constants.AutoConstants.AUTO_EVENT_MAP),

        new FollowPathWithEvents(
            driveSubsystem.followTrajectoryCommand(m_balancePath, false),
            m_balancePath.getMarkers(),
            Constants.AutoConstants.AUTO_EVENT_MAP),
        new DriveToLevel(driveSubsystem)
            .alongWith(new RunCommand(() -> LED.cycle()))
        );
    }
}
