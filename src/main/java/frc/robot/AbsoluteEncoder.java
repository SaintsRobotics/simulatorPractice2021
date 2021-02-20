/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class AbsoluteEncoder {
    private AnalogInput analogIn;
    private AnalogInputSim analogInSim;
    private boolean isInverted;
    private double voltageToDegrees = 360 / 5;
    private double m_offset;

    /**
     * Construct and absolute encoder, most likely a US Digital MA3 encoder.
     * 
     * @param channel  analog in (sometimes also refered to as AIO) port on the
     *                 roboRIO
     * @param inverted set this to TRUE if physically turning the encoder CLOCKWISE
     *                 (looking down on it from the top of the bot) INCREASES the
     *                 voltage it returns
     * @param offset   swerve offset (in DEGREES), like we've been using for the
     *                 past three years. This value is SUBTRACTED from the raw
     *                 output
     */
    public AbsoluteEncoder(int channel, boolean inverted, double offset) {
        analogIn = new AnalogInput(channel);
        analogInSim = new AnalogInputSim(analogIn);
        analogInSim.setVoltage(0);
        isInverted = inverted;
        m_offset = offset;
    }

    /**
     * Does some fancy mafs (mathematics) to figure out the number that the
     * simulated absolute encoder should be reading. Read comments in source code
     * below.
     * 
     * @param turnVoltage voltage that will go to the turning motor, range: [-1, 1]
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
        double outputVoltage = analogIn.getVoltage() + voltsSinceLastTick;

        SmartDashboard.putNumber("turning voltage" + analogIn.getChannel(), turnVoltage);

        // convert output to a number between 0 and 5
        outputVoltage = (((outputVoltage % 5) + 5) % 5);

        // basically this hijacks the simulated absolute encoder to say that it's
        // reading the voltage that u give it, range: [0, 5]
        analogInSim.setVoltage(outputVoltage);

    }

    /**
     * 
     * @return the position of the wheel, in degrees. zero points toward the front
     *         of the bot
     */
    public double getDegrees() {
        if (isInverted) {
            return (5 - analogIn.getVoltage() - m_offset) * voltageToDegrees;
        }

        return (analogIn.getVoltage() - m_offset) * voltageToDegrees;

    }

    public double getRadians() {
        return Math.toRadians(getDegrees());
    }
}