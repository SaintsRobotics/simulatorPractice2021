// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Moves the robot a specified distance in a specified field relative direction.
 */
public class MoveDirectionCommand extends GoToPositionCommand {
    private double m_direction = 0;
    private double m_distance = 0;

    /**
     * Creates a new {@link MoveDirectionCommand}.
     * 
     * @param drivetrain The {@link SwerveDrivetrain} subsystem to use.
     */
    public MoveDirectionCommand(SwerveDrivetrain drivetrain) {
        super(drivetrain);
    }

    @Override
    public void initialize() {
        m_targetX = m_drivetrain.getCurrentPosition().getX() + Math.cos(m_direction) * m_distance;
        m_targetY = m_drivetrain.getCurrentPosition().getY() + Math.sin(m_direction) * m_distance;
        super.initialize();
    }

    /**
     * Updates the command with a new direction to move.
     * 
     * @param direction The desired field relative direction to move.
     * @return The updated {@link MoveDirectionCommand} for method chaining.
     */
    public MoveDirectionCommand withDirection(double direction) {
        m_direction = direction;
        return this;
    }

    /**
     * Updates the command with a new distance to travel.
     * 
     * @param distance The desired distance to travel.
     * @return The updated {@link MoveDirectionCommand} for method chaining.
     */
    public MoveDirectionCommand withDistance(double distance) {
        m_distance = distance;
        return this;
    }

    /**
     * Updates the command with a new final target heading.
     * 
     * @param rotation The desired final field relative heading.
     * @return The updated {@link MoveDirectionCommand} for method chaining.
     */
    public MoveDirectionCommand withHeading(double rotation) {
        m_targetRotation = rotation;
        return this;
    }
}