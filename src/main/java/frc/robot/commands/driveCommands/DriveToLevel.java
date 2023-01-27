package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToLevel extends CommandBase {
    private DriveSubsystem m_drivetrain;

    public DriveToLevel(DriveSubsystem drivetrain){
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute(){
        m_drivetrain.driveToLevel();
    }

    @Override
    public boolean isFinished(){
        return m_drivetrain.isLevel();
    }
}
