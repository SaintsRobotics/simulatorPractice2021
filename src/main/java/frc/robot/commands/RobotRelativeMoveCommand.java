/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Moves robot a predefined x and y displacement
 */
public class RobotRelativeMoveCommand extends GoToPositionCommand {
    private double m_xChange = 0;
    private double m_yChange = 0;

    /**
     * Constructs the command.
     * 
     * @param drivetrain required subsystem
     */
    public RobotRelativeMoveCommand(SwerveDrivetrain drivetrain) {
        super(drivetrain);
        // TODO Auto-generated constructor stub
    }

    @Override
    /**
     * Sets target setpoints for all three PIDs. Target X position is the current
     * position plus the change in x Target Y position is the current position plus
     * the change in y Rotation does not change.
     */
    public void initialize() {
        m_targetX = m_drivetrain.getCurrentPosition().getX() + m_xChange;
        m_targetY = m_drivetrain.getCurrentPosition().getY() + m_yChange;
        super.initialize();
    }

    /**
     * Sets the target change in x.
     * 
     * @param x the desired change in x
     * @return returning the object allows for method chaining.
     */
    public RobotRelativeMoveCommand withChangeInX(double x) {
        m_xChange = x;
        return this;
    }

    /**
     * Sets the target change in y.
     * 
     * @param y the desired change in y
     * @return returning the object allows for method chaining.
     */
    public RobotRelativeMoveCommand withChangeInY(double y) {
        m_yChange = y;
        return this;
    }

    /**
     * Sets the target change in rotation.
     * 
     * @param r the desired change in rotation
     * @return returning the object allows for method chaining.
     */
    public RobotRelativeMoveCommand withHeading(double r) {
        m_targetRotation = r;
        return this;
    }
}
