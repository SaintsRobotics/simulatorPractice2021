// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/** Runs the intake in reverse to get rid of balls. */
public class OuttakeCommand extends CommandBase {
  private Intake m_intakeSubsystem;

  /**
   * Creates a new {@link OuttakeCommand}.
   * 
   * @param intake The {@link Intake} subsystem to use.
   */
  public OuttakeCommand(Intake intake) {
    m_intakeSubsystem = intake;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void execute() {
    m_intakeSubsystem.outtake();
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
  }
}