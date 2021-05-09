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
    }

    @Override
    public void initialize() {
        m_targetX = m_drivetrain.getCurrentPosition().getX() + m_xChange;
        m_targetY = m_drivetrain.getCurrentPosition().getY() + m_yChange;
        super.initialize();
    }

    /**
     * Sets the target change in x.
     * 
     * @param xChange the desired change in x
     * @return returning the object allows for method chaining.
     */
    public RobotRelativeMoveCommand withChangeInX(double xChange) {
        m_xChange = xChange;
        return this;
    }

    /**
     * Sets the target change in y.
     * 
     * @param yChange the desired change in y
     * @return returning the object allows for method chaining.
     */
    public RobotRelativeMoveCommand withChangeInY(double yChange) {
        m_yChange = yChange;
        return this;
    }

     /**
     * Sets the target change in rotation.
     * 
     * @param targetRotation the desired change in rotation
     * @return returning the object allows for method chaining.
     */
    public RobotRelativeMoveCommand withHeading(double targetRotation) {
        m_targetRotation = targetRotation;
        return this;
    }
}
