// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/** Moves the robot to a field relative position. */
public class FieldRelativeMoveCommand extends GoToPositionCommand {
	/**
	 * Creates a new {@link FieldRelativeMoveCommand}.
	 * 
	 * @param drivetrain The {@link SwerveDrivetrain} subsystem to use.
	 */
	public FieldRelativeMoveCommand(SwerveDrivetrain drivetrain) {
		super(drivetrain);
	}

	/**
	 * Updates the command with a new X position.
	 * 
	 * @param targetX Target field relative X position.
	 * @return The updated {@link FieldRelativeMoveCommand} for method chaining.
	 */
	public FieldRelativeMoveCommand withX(double targetX) {
		m_targetX = targetX;
		return this;
	}

	/**
	 * Updates the command with a new Y position.
	 * 
	 * @param targetY Target field relative Y position.
	 * @return The updated {@link FieldRelativeMoveCommand} for method chaining.
	 */
	public FieldRelativeMoveCommand withY(double targetY) {
		m_targetY = targetY;
		return this;
	}

	/**
	 * Updates the command with a new heading.
	 * 
	 * @param targetRotation Target field relative heading.
	 * @return The updated {@link FieldRelativeMoveCommand} for method chaining.
	 */
	public FieldRelativeMoveCommand withHeading(double targetRotation) {
		m_targetRotation = targetRotation;
		return this;
	}
}