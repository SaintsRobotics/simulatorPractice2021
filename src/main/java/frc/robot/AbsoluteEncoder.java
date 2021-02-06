/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Add your docs here.
 */
public class AbsoluteEncoder {
    private AnalogInput analogIn;
    private boolean isInverted;
    private double voltageToDegrees = 360 / 5;
    private double m_offset;

    public AbsoluteEncoder(int channel, boolean inverted, double offset) {
        analogIn = new AnalogInput(channel);
        isInverted = inverted;
        m_offset = offset;
    }

    public void sendVoltage(double turnVoltage) {
        double ratio = 5.0;
        if(turnVoltage > 1){
            turnVoltage = 1;
        } else if (turnVoltage < -1){
            turnVoltage = -1;
        }
        // starting position (in volts), add turnVoltage * gear ratio (V/s), multiply by scale factor for units
        double output = turnVoltage;
        return output;
    }

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