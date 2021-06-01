package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Command to move the robot one meter in a designated direction
 */
public class MoveOneMeterCommand extends GoToPositionCommand {
  private double m_direction;

  /**
   * Creates a new MoveOneMeterCommand.
   */
  public MoveOneMeterCommand(SwerveDrivetrain drivetrain) {
    super(drivetrain);
  }

  @Override
  public void initialize() {
    m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX() + Math.cos(m_direction));
    m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY() + Math.sin(m_direction));
    m_rotationPID.setSetpoint(m_direction);
  }

  /**
   * Sets the desired robot direction (field relative)
   * 
   * @param direction Desired robot direction
   * @return Updated command
   */
  public MoveOneMeterCommand withDirection(double direction) {
    m_direction = direction;
    return this;
  }
}
