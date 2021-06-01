package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Moves the robot to a field relative position.
 */
public class FieldRelativeMoveCommand extends GoToPositionCommand {

  /**
   * Creates a new FieldRelativeMoveCommand.
   */
  public FieldRelativeMoveCommand(SwerveDrivetrain drivetrain) {
    super(drivetrain);
  }

  /**
   * Updates the command with a new X position.
   * 
   * @param targetX Target field relative X position.
   * @return Returns updated FieldRelativeCommand.
   */
  public FieldRelativeMoveCommand withX(double targetX) {
    m_targetX = targetX;
    return this;
  }

  /**
   * Updates the command with a new Y position.
   * 
   * @param targetY Target field relative Y position.
   * @return Returns updated FieldRelativeCommand.
   */
  public FieldRelativeMoveCommand withY(double targetY) {
    m_targetY = targetY;
    return this;
  }

  /**
   * Updates the command with a new heading.
   * 
   * @param targetRotation Target field relative heading.
   * @return Returns updated FieldRelativeCommand.
   */
  public FieldRelativeMoveCommand withHeading(double targetRotation) {
    m_targetRotation = targetRotation;
    return this;
  }
}
