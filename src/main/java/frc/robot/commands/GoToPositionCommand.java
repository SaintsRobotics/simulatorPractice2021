// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Parent class for moving the robot. Child classes pass in targetX, targetY,
 * and targetRotation setpoints. If nothing is passed in, the current position
 * is used as the setpoint.
 */
public class GoToPositionCommand extends CommandBase {
	protected SwerveDrivetrain m_drivetrain;
	protected PIDController m_xPID;
	protected PIDController m_yPID;
	protected PIDController m_rotationPID;
	private Pose2d m_currentPosition;
	protected double m_targetX = Constants.defaultNull;
	protected double m_targetY = Constants.defaultNull;
	protected double m_targetRotation = Constants.defaultNull;
	private int m_counter;

	/**
	 * Creates a new {@link GoToPositionCommand}.
	 * 
	 * @param drivetrain The {@link SwerveDrivetrain} subsystem to use.
	 */
	public GoToPositionCommand(SwerveDrivetrain drivetrain) {
		addRequirements(drivetrain);
		m_drivetrain = drivetrain;

		m_xPID = new PIDController(Constants.SwerveConstants.MAX_METERS_PER_SECOND, 0, 0);
		m_yPID = new PIDController(Constants.SwerveConstants.MAX_METERS_PER_SECOND, 0, 0);
		m_rotationPID = new PIDController(Math.PI * 6, 0, 0);

		m_xPID.setTolerance(0.05);
		m_yPID.setTolerance(0.05);
		m_rotationPID.setTolerance(Math.PI / 24);

		m_rotationPID.enableContinuousInput(-Math.PI, Math.PI);
	}

	public void initialize() {
		if (m_targetX != Constants.defaultNull) {
			m_xPID.setSetpoint(m_targetX);
		} else {
			m_xPID.setSetpoint(m_drivetrain.getPose().getX());
		}

		if (m_targetY != Constants.defaultNull) {
			m_yPID.setSetpoint(m_targetY);
		} else {
			m_yPID.setSetpoint(m_drivetrain.getPose().getY());
		}

		if (m_targetRotation != Constants.defaultNull) {
			m_rotationPID.setSetpoint(m_targetRotation);
		} else {
			m_rotationPID.setSetpoint(m_drivetrain.getPose().getRotation().getRadians());
		}
	}

	@Override
	public void execute() {
		m_currentPosition = m_drivetrain.getPose();
		m_drivetrain.drive(m_xPID.calculate(m_currentPosition.getX()), m_yPID.calculate(m_currentPosition.getY()),
				-m_rotationPID.calculate(m_currentPosition.getRotation().getRadians()), true);
	}

	@Override
	public void end(boolean interrupted) {
		m_drivetrain.drive(0, 0, 0, false);
	}

	@Override
	public boolean isFinished() {
		if (m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotationPID.atSetpoint()) {
			m_counter++;
		} else {
			m_counter = 0;
		}
		return m_counter > 10;
	}
}