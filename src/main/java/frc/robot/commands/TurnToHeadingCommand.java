/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

public class TurnToHeadingCommand extends GoToPositionCommand {
    private double m_targetHeading = m_drivetrain.getCurrentPosition().getRotation().getRadians();

    /**
     * Constructs the command.
     * 
     * @param drivetrain required subsystem
     */
    public TurnToHeadingCommand(SwerveDrivetrain drivetrain) {
        super(drivetrain);
    }

    @Override
    /**
     * Sets target setpoints for all three PIDs.Target rotation is value that is fed in, x and y positions should not change.
     */
    public void initialize() {
        // TODO Auto-generated method stub
        m_rotationPID.setSetpoint(m_targetHeading);
        m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY());
        m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX());

    }
    
    /**
     * Sets the target rotation.
     * 
     * @param targetHeading the desired heading (a value in radians)
     * @return returning the object allows for method chaining.
     */
    public TurnToHeadingCommand withRotation (double targetHeading){
        m_targetHeading = targetHeading;
        return this;
    }
    

}