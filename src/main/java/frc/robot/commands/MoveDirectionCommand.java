/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

public class MoveDirectionCommand extends GoToPositionCommand {
    private double m_xChange = 0;
    private double m_yChange = 0;

    /**
     * Constructs the command.
     * 
     * @param drivetrain required subsystem
     */
    public MoveDirectionCommand(SwerveDrivetrain drivetrain) {
        super(drivetrain);
        // TODO Auto-generated constructor stub
    }

    @Override
    /**
     * Sets target setpoints for all three PIDs. 
     * Target X position is the current position plus the change in x
     * Target Y position is the current position plus the change in y
     * Rotation does not change.
     */
    public void initialize() {
        m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX() + m_xChange);
        m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY() + m_yChange);
        m_rotationPID.setSetpoint(m_drivetrain.getCurrentPosition().getRotation().getRadians());

    }
    
    /**
     * Sets the target change in x.
     * 
     * @param x the desired change in x 
     * @return returning the object allows for method chaining.
     */
    public MoveDirectionCommand withChangeInX (double x) {
        m_xChange = x;
        return this;
    }

    /**
     * Sets the target change in y.
     * 
     * @param y the desired change in y 
     * @return returning the object allows for method chaining.
     */
    public MoveDirectionCommand withChangeInY (double y) {
        m_yChange = y;
        return this;
    }
}