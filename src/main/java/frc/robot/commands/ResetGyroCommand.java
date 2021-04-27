package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class ResetGyroCommand extends CommandBase {
    
    private SwerveDrivetrain m_drivetrain;

    public ResetGyroCommand(SwerveDrivetrain drivetrain) {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        m_drivetrain.resetGyro();
    }
}
