// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Absolute encoder. */
public class AbsoluteEncoder {
	private AnalogInput m_analogIn;
	private AnalogInputSim m_analogSim;
	private boolean m_isInverted;
	private double m_voltageToRadians = Math.PI * 2 / 5;
	private double m_offset;
	private int m_polarity;

	/**
	 * Construct and absolute encoder, most likely a US Digital MA3 encoder.
	 * 
	 * @param channel  Analog in (sometimes also refered to as AIO) port on the
	 *                 roboRIO.
	 * @param inverted Set this to <i>TRUE</i> if physically turning the wheel
	 *                 <i>CLOCKWISE</i> (looking down on it from the top of the bot)
	 *                 <i>INCREASES</i> the voltage it returns.
	 * @param offset   Swerve offset (in <i>DEGREES</i>), like we've been using for
	 *                 the past three years. This value is <i>SUBTRACTED</i> from
	 *                 the output.
	 */
	public AbsoluteEncoder(int channel, boolean inverted, double offset) {
		m_analogIn = new AnalogInput(channel);
		m_analogSim = new AnalogInputSim(m_analogIn);
		m_analogSim.setVoltage(0);
		m_isInverted = inverted;
		m_offset = offset;
		m_polarity = m_isInverted ? -1 : 1;
	}

	/**
	 * Figures out the value that the simulated absolute encoder should be at. Read
	 * comments in source code below.
	 * 
	 * @param turnVoltage Voltage that will go to the turning motor, range: [-1, 1].
	 */
	public void sendVoltage(double turnVoltage) {
		// gear ratio between motor and wheel / encoder
		double gearRatio = 12.8;

		// maximum RPM for motor under 0 load
		double motorRPM = 5500;

		// how long it takes to run one loop of the periodic (in seconds)
		double tickPeriod = Robot.kDefaultPeriod;

		if (turnVoltage > 1) {
			turnVoltage = 1;
		} else if (turnVoltage < -1) {
			turnVoltage = -1;
		}

		// started with motorRPM and converted to wheel rotations per tick
		// RPM * (min / sec) * (s / tick) * (wheel rotations / motor rotations)
		double wheelRotationsPerTick = motorRPM / 60 * tickPeriod / gearRatio; // 0.143
		double wheelRotationsSinceLastTick = wheelRotationsPerTick * turnVoltage;
		double voltsSinceLastTick = 5 * wheelRotationsSinceLastTick;
		voltsSinceLastTick *= m_polarity;

		double outputVoltage = m_analogIn.getVoltage() + voltsSinceLastTick;

		SmartDashboard.putNumber("turning voltage" + m_analogIn.getChannel(), turnVoltage);

		// convert output to a number between 0 and 5
		outputVoltage = (((outputVoltage % 5) + 5) % 5);

		// basically this "hijacks" the simulated absolute encoder to say that it's
		// reading the voltage that u give it, range: [0, 5]
		m_analogSim.setVoltage(outputVoltage);
	}

	/**
	 * 
	 * @return The position of the as a Rotation2d. Zero points toward the front of
	 *         the bot. The value increases as the swerve wheel is turned clockwise.
	 */
	public Rotation2d getAngle() {
		if (m_isInverted) {
			return new Rotation2d((5 - m_analogIn.getVoltage()) * m_voltageToRadians - m_offset);
		}
		return new Rotation2d((m_analogIn.getVoltage()) * m_voltageToRadians - m_offset);
	}
}