package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

public class TurnToHeadingCommand extends GoToPositionCommand {

  private double m_targetRotation;

  public TurnToHeadingCommand(SwerveDrivetrain drivetrain) {
    super(drivetrain);
  }

  @Override
  public void initialize() {
    m_rotationPID.setSetpoint(m_targetRotation);
  }

  public TurnToHeadingCommand withRotation(double targetRotation) {
    m_targetRotation = targetRotation;
    return this;
  }
}
