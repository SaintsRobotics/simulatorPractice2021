/*---------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


/* ideas for default to current pos
    1. use boolean flags when "with" commands are called combined with if statements in initialize
         (might use two arrays to keep things neat instead of having like 6 variables)
    2. could we use the thing where you passed in methods to use as values so they could be updated
        so like instead of initially declaring them and setting them as the current VALUE
        we could declare them and set them to be the most updated result of *this method*
    3. how would we feel about gotoposition having a getcurrentpos method and a gettargetpos method
        in gotoposition the targetpos is by default the currentpos
        but child methods can override if necessary
*/
package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

/**
* Turns robot to field relative heading
*/
public class TurnToHeadingCommand extends GoToPositionCommand {
    /**
     * Constructs the command.
     * 
     * @param drivetrain required subsystem
     */
    public TurnToHeadingCommand(SwerveDrivetrain drivetrain) {
        super(drivetrain);
    }

    /**
     * Sets the target rotation.
     * 
     * @param targetHeading the desired heading (a value in radians)
     * @return returning the object allows for method chaining.
     */
    public TurnToHeadingCommand withHeading (double targetHeading){
        m_targetRotation = targetHeading;
        return this;
    }
    

}

 
 

    