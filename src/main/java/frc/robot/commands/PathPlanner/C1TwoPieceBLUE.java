package frc.robot.commands.PathPlanner;

/* EXTENDED PATH ONTO RIT CHARGESTATION BY 8" */

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.commands.driveCommands.DriveToLevel;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;

public class C1TwoPieceBLUE extends SequentialCommandGroup {

    public C1TwoPieceBLUE(DriveSubsystem driveSubsystem, 
                        ArmSubsystem armSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LEDs LED
                        ){

        PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Cone Score 1", 
            new PathConstraints(2, 3));
        // PathPlannerTrajectory m_secondPath = PathPlanner.loadPath("Cone Reverse 1", 
        //     new PathConstraints(2, 1));
        PathPlannerTrajectory m_thirdPath = PathPlanner.loadPath("Cube to 2 from 1 Grid", 
            new PathConstraints(2, 2.2));
        PathPlannerTrajectory m_forthPath = PathPlanner.loadPath("Balance from 2 BLUE", 
            new PathConstraints(2.5, 2.5));
        // PathPlannerTrajectory m_fifthPath = PathPlanner.loadPath("New Drive to Cube 9", 
        //     new PathConstraints(2, 3));
        // PathPlannerTrajectory m_sixthPath = PathPlanner.loadPath("Go to 8 with Cube", 
        //     new PathConstraints(1, 2));

        
        addCommands(
            new InstantCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kIntakePickUp), intakeSubsystem),
            new ArmToPositionWithEnd(armSubsystem, armPositions.LVLTRE).withTimeout(1.40),
            new FollowPathWithEvents(
                driveSubsystem.followTrajectoryCommand(m_firstPath, true),
                m_firstPath.getMarkers(),
                Constants.AutoConstants.AUTO_EVENT_MAP),
            new RunCommand(() -> intakeSubsystem.intakeOn(Constants.IntakeConstants.kIntakeReverse), intakeSubsystem).withTimeout(.25),
            // new WaitCommand(0.5),
            // driveSubsystem.followTrajectoryCommand(m_secondPath, false),
            // new InstantCommand(() -> intakeSubsystem.intakeOff()),
            new FollowPathWithEvents(
                driveSubsystem.followTrajectoryCommand(m_thirdPath, false),
                m_thirdPath.getMarkers(),
                Constants.AutoConstants.AUTO_EVENT_MAP),
            new RunCommand(() -> intakeSubsystem.intakeOn(IntakeConstants.kIntakePickUp), intakeSubsystem).withTimeout(0.25)
                .alongWith(new InstantCommand(() -> LED.setColor(LED.BLUE))), // TODDO: improve intake constant names
            new FollowPathWithEvents(
                driveSubsystem.followTrajectoryCommand(m_forthPath, false),
                m_forthPath.getMarkers(),                                                   //No markers
                Constants.AutoConstants.AUTO_EVENT_MAP),
            new DriveToLevel(driveSubsystem)
                .alongWith(new RunCommand(() -> LED.cycle()))
            
            // new ParallelCommandGroup(new ArmToPositionWithEnd(armSubsystem, armPositions.HOME).withTimeout(2.0),
            //     driveSubsystem.followTrajectoryCommand(m_thirdPath, true))
            // new DriveToLevel(driveSubsystem)
            );
            // TODO: why does groundJoint come out with groundIntake On right before climbing Charge Station?
            // (starts at waypoint 2 of "Balance from 2" path)
    }
}
