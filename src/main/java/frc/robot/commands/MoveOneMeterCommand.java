package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

public class MoveOneMeterCommand extends GoToPositionCommand {

  private double m_heading;

  public MoveOneMeterCommand(SwerveDrivetrain drivetrain) {
    super(drivetrain);
  }

  @Override
  public void initialize() {
    m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX() + Math.cos(m_heading));
    m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY() + Math.sin(m_heading));
  }

  public MoveOneMeterCommand withHeading(double heading) {
    m_heading = heading;
    return this;
  }
}
