// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/** Moves robot a predefined x and y displacement. */
public class RobotRelativeMoveCommand extends GoToPositionCommand {
	private double m_xChange = 0;
	private double m_yChange = 0;

	/**
	 * Creates a new {@link RobotRelativeMoveCommand}.
	 * 
	 * @param drivetrain The {@link SwerveDrivetrain} subsystem to use.
	 */
	public RobotRelativeMoveCommand(SwerveDrivetrain drivetrain) {
		super(drivetrain);
	}

	@Override
	public void initialize() {
		m_targetX = m_drivetrain.getPose().getX() + m_xChange;
		m_targetY = m_drivetrain.getPose().getY() + m_yChange;
		super.initialize();
	}

	/**
	 * Sets the target change in x.
	 * 
	 * @param xChange The desired change in x.
	 * @return The updated {@link RobotRelativeMoveCommand} for method chaining.
	 */
	public RobotRelativeMoveCommand withChangeInX(double xChange) {
		m_xChange = xChange;
		return this;
	}

	/**
	 * Sets the target change in y.
	 * 
	 * @param yChange The desired change in y.
	 * @return The updated {@link RobotRelativeMoveCommand} for method chaining.
	 */
	public RobotRelativeMoveCommand withChangeInY(double yChange) {
		m_yChange = yChange;
		return this;
	}

	/**
	 * Sets the target change in rotation.
	 * 
	 * @param targetRotation The desired change in rotation.
	 * @return The updated {@link RobotRelativeMoveCommand} for method chaining.
	 */
	public RobotRelativeMoveCommand withHeading(double targetRotation) {
		m_targetRotation = targetRotation;
		return this;
	}
}