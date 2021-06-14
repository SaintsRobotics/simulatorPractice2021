// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Command to move the robot one meter in a designated direction
 */
public class MoveOneMeterCommand extends GoToPositionCommand {
  /** Creates a new MoveOneMeterCommand. */
  private double m_targetR;

  public MoveOneMeterCommand(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX() + Math.cos(m_targetR));
    m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY() + Math.sin(m_targetR));
    m_rotationPID.setSetpoint(m_targetR);

  }

  /**
   * Sets the desired robot direction (field relative)
   * @param tR Desired robot direction
   * @return Updated command
   */
  public MoveOneMeterCommand withR(double tR){
    m_targetR = tR;
    return this;
  }
}
