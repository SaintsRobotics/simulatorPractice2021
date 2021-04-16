// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class TurnToHeadingCommand extends GoToPositionCommand {
  private double m_targetR = -1000;

  /** Creates a new TurnToHeadingCommand. */
  public TurnToHeadingCommand(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX());
    m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY());
    if (m_targetR != -1000) {
      m_rotationPID.setSetpoint(m_targetR);
    } else {
      m_rotationPID.setSetpoint(m_drivetrain.getCurrentPosition().getRotation().getRadians());
    }
  }

  public TurnToHeadingCommand withR(double r) {
    m_targetR = r;
    return this;
  }

}
