/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

public class MoveOneMeterCommand extends GoToPositionCommand {
    private double m_direction = m_drivetrain.getCurrentPosition().getRotation().getRadians();

    /**
     * Constructs the command.
     * 
     * @param drivetrain required subsystem
     */
    public MoveOneMeterCommand(SwerveDrivetrain drivetrain) {
        super(drivetrain);
        // TODO Auto-generated constructor stub
    }

    @Override
    /**
     * Sets target setpoints for all three PIDs. 
     * Target X position is the current position plus the x component of one meter in a specified direction
     * Target Y position is the current position plus the y component of one meter in a specified direction
     * Rotation does not change.
     */
    public void initialize() {
        m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX() + Math.cos(m_direction));
        m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY() + Math.sin(m_direction));
        m_rotationPID.setSetpoint(m_drivetrain.getCurrentPosition().getRotation().getRadians());
    }

    /**
     * Specifies the target direction.
     * 
     * @param direction the desired direction (a value in radians)
     * @return returning the object allows for method chaining.
     */

    public MoveOneMeterCommand withDirection (double direction) {
        m_direction = direction;
        return this;
    }
    

}