// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase implements AutoCloseable {
	private Intake m_intakeSubsystem;

	/**
	 * Creates a new {@link IntakeCommand}.
	 * 
	 * @param intake The {@link Intake} subsystem to use.
	 */
	public IntakeCommand(Intake intake) {
		m_intakeSubsystem = intake;
		addRequirements(m_intakeSubsystem);
	}

	@Override
	public void initialize() {
		m_intakeSubsystem.intake();
	}

	@Override
	public void end(boolean interrupted) {
		m_intakeSubsystem.stopIntake();
	}

	@Override
	public void close() throws Exception {
		// Do this soon
	}
}