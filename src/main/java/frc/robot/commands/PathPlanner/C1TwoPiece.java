package frc.robot.commands.PathPlanner;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.commands.driveCommands.DriveToLevel;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;

public class C1TwoPiece extends SequentialCommandGroup {

    public C1TwoPiece(
        boolean isRED,
        DriveSubsystem driveSubsystem, 
        ArmSubsystem armSubsystem,
        IntakeSubsystem intakeSubsystem,
        LEDs LED
        ){

    PathPlannerTrajectory m_conePath = PathPlanner.loadPath("Cone Score 1", 
        new PathConstraints(2, 3));
    PathPlannerTrajectory m_pickupPath = PathPlanner.loadPath("Cube to 2 from 1 Grid", 
        new PathConstraints(2, 2.2));
    PathPlannerTrajectory m_balancePath;

    if (isRED)
        m_balancePath = PathPlanner.loadPath("Balance from 2 RED", 
            new PathConstraints(2.5, 2.5));
    else 
        m_balancePath = PathPlanner.loadPath("Balance from 2 BLUE", 
            new PathConstraints(2.5, 2.5));
    
    addCommands(
        new InstantCommand(() -> intakeSubsystem.intakeOn(IntakeConstants.kConePickUp), intakeSubsystem),
        new ArmToPositionWithEnd(armSubsystem, armPositions.LVLTRE).withTimeout(1.20),
        new FollowPathWithEvents(
            driveSubsystem.followTrajectoryCommand(m_conePath, true),
            m_conePath.getMarkers(),
            AutoConstants.AUTO_EVENT_MAP),
        new RunCommand(() -> intakeSubsystem.intakeOn(IntakeConstants.kConeEject), intakeSubsystem).withTimeout(.25),

        new FollowPathWithEvents(
            driveSubsystem.followTrajectoryCommand(m_pickupPath, false),
            m_pickupPath.getMarkers(),
            AutoConstants.AUTO_EVENT_MAP),
        new RunCommand(() -> intakeSubsystem.intakeOn(IntakeConstants.kCubeEject), intakeSubsystem).withTimeout(0.25)   // TODO: Possible to Reduce?

            .alongWith(new InstantCommand(() -> LED.setColor(LED.BLUE))),
        new FollowPathWithEvents(
            driveSubsystem.followTrajectoryCommand(m_balancePath, false),
            m_balancePath.getMarkers(),
            AutoConstants.AUTO_EVENT_MAP),
        new DriveToLevel(driveSubsystem)
            .alongWith(new RunCommand(() -> LED.cycle()))
        );
    }
}
