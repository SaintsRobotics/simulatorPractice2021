package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class ResetGyroCommand extends CommandBase {
    
    private SwerveDrivetrain m_drivetrain;

    public ResetGyroCommand(SwerveDrivetrain drivetrain) {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drivetrain.resetGyro();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}
