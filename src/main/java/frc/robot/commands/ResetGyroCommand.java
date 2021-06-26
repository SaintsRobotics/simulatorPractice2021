// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

/** Resets the gyro of the drivetrain. */
public class ResetGyroCommand extends CommandBase {
	private SwerveDrivetrain m_drivetrain;

	/**
	 * Creates a new {@link ResetGyroCommand}.
	 * 
	 * @param drivetrain The {@link SwerveDrivetrain} subsystem to use.
	 */
	public ResetGyroCommand(SwerveDrivetrain drivetrain) {
		addRequirements(drivetrain);
		m_drivetrain = drivetrain;
	}

	@Override
	public void initialize() {
		m_drivetrain.resetGyro();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}