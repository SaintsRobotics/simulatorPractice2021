// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/** Turns the robot to field relative heading. */
public class TurnToHeadingCommand extends GoToPositionCommand {
	/**
	 * Constructs the command.
	 * 
	 * @param drivetrain required subsystem
	 */
	public TurnToHeadingCommand(SwerveDrivetrain drivetrain) {
		super(drivetrain);
	}

	/**
	 * Sets the target heading.
	 * 
	 * @param targetHeading The desired heading in radians.
	 * @return The updated {@link TurnToHeadingCommand} for method chaining.
	 */
	public TurnToHeadingCommand withHeading(double targetHeading) {
		m_targetRotation = targetHeading;
		return this;
	}
}