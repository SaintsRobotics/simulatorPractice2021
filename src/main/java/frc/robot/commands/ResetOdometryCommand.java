package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class ResetOdometryCommand extends CommandBase {

    private SwerveDrivetrain m_drivetrain;

    public ResetOdometryCommand(SwerveDrivetrain drivetrain) {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        m_drivetrain.resetOdometry();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
