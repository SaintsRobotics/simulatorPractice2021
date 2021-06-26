// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AbsoluteEncoder;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;

/** An individual swerve module. */
public class SwerveModule {
	private final CANSparkMax m_driveMotor;
	private final CANSparkMax m_turningMotor;

	private final AbsoluteEncoder m_turningEncoder;

	private final PIDController m_turningPIDController = new PIDController(0.3, 0, 0);

	private final Translation2d m_location;
	private final String m_name;
	
	private SwerveModuleState m_state;

	/**
	 * Constructs a {@link SwerveModule}.
	 * 
	 * @param name         The name of the {@link SwerveModule}.
	 * @param driveMotor   The motor controller for the drive motor.
	 * @param turningMotor The motor controller for the turning motor.
	 * @param x            The x position of the {@link SwerveModule}.
	 * @param y            The y position of the {@link SwerveModule}.
	 * @param encoder      The encoder for the {@link SwerveModule}.
	 */
	public SwerveModule(String name, CANSparkMax driveMotor, CANSparkMax turningMotor, double x, double y,
			AbsoluteEncoder encoder) {
		m_driveMotor = driveMotor;
		m_turningMotor = turningMotor;

		m_turningEncoder = encoder;

		m_location = new Translation2d(x, y);

		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

		m_name = name;
		m_state = new SwerveModuleState();
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param state Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState desiredState) {
		// Optimize the reference state to avoid spinning further than 90 degrees.
		SwerveModuleState state = SwerveModuleState.optimize(desiredState, m_turningEncoder.getAngle());

		final double driveOutput = state.speedMetersPerSecond / SwerveConstants.MAX_METERS_PER_SECOND;

		final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getAngle().getRadians(),
				state.angle.getRadians());

		if (Robot.isSimulation()) {
			m_turningEncoder.sendVoltage(turnOutput);
		}

		m_driveMotor.set(driveOutput);
		m_turningMotor.set(turnOutput);
		m_state = new SwerveModuleState(state.speedMetersPerSecond, m_turningEncoder.getAngle());

		SmartDashboard.putNumber(m_name + " Voltage", turnOutput);
		SmartDashboard.putNumber(m_name + " Current Angle", m_turningEncoder.getAngle().getDegrees());
		SmartDashboard.putNumber(m_name + " Desired Angle", state.angle.getDegrees());
		SmartDashboard.putNumber(m_name + " Error", m_turningPIDController.getPositionError());
	}

	/**
	 * Returns the current location of the module.
	 *
	 * @return The current location of the module.
	 */
	public Translation2d getLocation() {
		return m_location;
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		return m_state;
	}

	/**
	 * Sets the desired speed for the module.
	 *
	 * @param speed Desired speed.
	 */
	public void setVelocity(double speed) {
		m_state.speedMetersPerSecond = speed;
		m_driveMotor.set(speed / SwerveConstants.MAX_METERS_PER_SECOND);
		m_turningMotor.set(0);
	}
}