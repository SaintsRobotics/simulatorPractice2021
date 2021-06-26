// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Command to move the robot one meter in a designated direction.
 */
public class MoveOneMeterCommand extends GoToPositionCommand {
	private double m_direction;

	/**
	 * Creates a new {@link MoveOneMeterCommand}.
	 * 
	 * @param drivetrain The {@link SwerveDrivetrain} subsystem to use.
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
	 * Sets the desired field relative direction.
	 * 
	 * @param direction The desired field relative direction.
	 * @return The updated {@link MoveOneMeterCommand} for method chaining.
	 */
	public MoveOneMeterCommand withDirection(double direction) {
		m_direction = direction;
		return this;
	}
}