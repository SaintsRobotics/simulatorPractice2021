// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Command to rotate robot to a desired heading
 */
public class TurnToHeadingCommand extends GoToPositionCommand {
  /** Creates a new TurnToHeadingCommand. */
  private double m_targetR;


  public TurnToHeadingCommand(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotationPID.setSetpoint(m_targetR);
    //System.out.println(m_xPID.getSetpoint() + " " + m_yPID.getSetpoint());
    m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX());
    m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY());
  }

  /**
   * Updates desired heading of robot
   * @param tR Desired heading (field relative)
   * @return Updated command
   */
  public TurnToHeadingCommand withR(double tR){
    m_targetR = tR;
    return this;

  }

}
