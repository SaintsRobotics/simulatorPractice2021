package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

public class MoveDirectionCommand extends GoToPositionCommand {
  private double m_targetX;
  private double m_targetY;

  public MoveDirectionCommand(SwerveDrivetrain drivetrain) {
    super(drivetrain);
  }

  @Override
  public void initialize() {
    m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX()
        + ((m_targetX * Math.cos(m_drivetrain.getCurrentPosition().getRotation().getRadians()))
            + (m_targetY * Math.sin(-m_drivetrain.getCurrentPosition().getRotation().getRadians()))));
    m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY()
        + ((m_targetX * Math.sin(m_drivetrain.getCurrentPosition().getRotation().getRadians()))
            + (m_targetY * Math.cos(-m_drivetrain.getCurrentPosition().getRotation().getRadians()))));
    m_rotationPID.setSetpoint(m_drivetrain.getCurrentPosition().getRotation().getRadians());
  }
  
  public MoveDirectionCommand withX(double targetX) {
    m_targetX = targetX;
    return this;
  }

  public MoveDirectionCommand withY(double targetY) {
    m_targetY = targetY;
    return this;
  }
}
