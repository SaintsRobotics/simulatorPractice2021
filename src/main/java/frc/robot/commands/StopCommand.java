package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class StopCommand extends InstantCommand {

  private SwerveDrivetrain m_drivetrain;

  public StopCommand(SwerveDrivetrain drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    m_drivetrain.move(0, 0, 0, false);
  }
}
