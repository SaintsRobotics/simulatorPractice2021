package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Moves the robot a specified distance in a specified field relative direction
 */
public class MoveDirectionCommand extends GoToPositionCommand {
    private double m_direction = 0;
    private double m_distance = 0;

    /**
     * Constructs the command.
     * 
     * @param drivetrain required subsystem
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
     * Updates the command with a new direction.
     * 
     * @param direction The desired field relative direction.
     * @return Returns updated MoveDirectionCommand.
     */
    public MoveDirectionCommand withDirection(double direction) {
        m_direction = direction;
        return this;
    }

    /**
     * Updates the command with a new distance.
     * 
     * @param distance The desired distance to travel
     * @return Returns updated MoveDirectionCommand.
     */
    public MoveDirectionCommand withDistance(double distance) {
        m_distance = distance;
        return this;
    }

    /**
     * Updates the command with a new heading.
     * 
     * @param rotation Desired field relative heading
     * @return Returns updated MoveDirectionCommand.
     */
    public MoveDirectionCommand withHeading(double rotation) {
        m_targetRotation = rotation;
        return this;
    }
}
