// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrivetrain;

/** Stops the drivetrain. */
public class StopCommand extends InstantCommand {
  private SwerveDrivetrain m_drivetrain;

  /**
   * Creates a new {@link StopCommand}.
   * 
   * @param drivetrain The {@link SwerveDrivetrain} subsystem to use.
   */
  public StopCommand(SwerveDrivetrain drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    m_drivetrain.move(0, 0, 0, false);
  }
}