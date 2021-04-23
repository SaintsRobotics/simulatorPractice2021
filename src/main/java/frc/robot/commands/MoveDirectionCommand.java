/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/**
* Moves a specified distance in a specified field relative direction
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
        // TODO Auto-generated constructor stub
    }

    @Override
    /**
     * Sets target setpoints for all three PIDs. 
     * Target X position is the current position plus the x component of the desired displacement in a specified direction
     * Target Y position is the current position plus the y component of the desired displacement in a specified direction
     * Rotation can be set in withHeading()
     */
    public void initialize() {
        m_targetX = m_drivetrain.getCurrentPosition().getX() + Math.cos(m_direction) * m_distance;
        m_targetY = m_drivetrain.getCurrentPosition().getY() + Math.sin(m_direction) * m_distance;
        super.initialize();
    }

    /**
     * Specifies the target direction - the direction in which the robot travels
     * 
     * @param direction the desired direction (a value in radians)
     * @return returning the object allows for method chaining
     */

    public MoveDirectionCommand withDirection (double direction) {
        m_direction = direction;
        return this;
    }

   /**
   * Specifies distance to travel
   * @param distance the desired distance to travel
   * @return returning the object allows for method chaining
   */
    public MoveDirectionCommand withDistance (double distance) {
        m_distance = distance;    
        return this;
    }

   /**
   * Changes robot heading - the angle the robot should turn to
   * @param rotation desired field relative heading
   * @return returning the object allows for method chaining
   */
    public MoveDirectionCommand withHeading (double rotation) {
        m_targetRotation = rotation;    
        return this;
    }
    

}
