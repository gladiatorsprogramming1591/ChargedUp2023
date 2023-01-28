package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToAngle extends CommandBase {
    private DriveSubsystem m_drivetrain;
    private boolean atAngle = false;

    public DriveToAngle(DriveSubsystem drivetrain){
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    public void initialize(){
        atAngle = false;
    }

    @Override
    public void execute(){
        atAngle = m_drivetrain.driveToAngle(Constants.AutoConstants.kDriveAngle);
    }

    @Override
    public boolean isFinished(){
        return atAngle;
    }
}
