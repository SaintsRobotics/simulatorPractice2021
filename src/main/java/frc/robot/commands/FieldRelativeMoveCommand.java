// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Class to autonomously move the robot to a designated field relative position 
 */
public class FieldRelativeMoveCommand extends GoToPositionCommand {
  /** Creates a new FieldRelativeMoveCommand. */
  // private double m_targetX = m_drivetrain.getCurrentPosition().getX();
  // private double m_targetY = m_drivetrain.getCurrentPosition().getY();
  // private double m_targetR = m_drivetrain.getCurrentPosition().getRotation().getRadians();

  public FieldRelativeMoveCommand(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain);
  }

  /**
   * Updates command with new parameters
   * @param tX Target field relative X position
   * @return Returns updated FieldRelativeCommand 
   */
  public FieldRelativeMoveCommand withX(double tX){ //e.g. creates command with some "x"
    m_targetX = tX;
    return this;
  }

  /**
   * Updates command with new parameters
   * @param tY Target field relative Y position
   * @return Returns updated FieldRelativeCommand 
   */
  public FieldRelativeMoveCommand withY(double tY){
    m_targetY = tY;
    return this;

  }

   /**
   * Updates command with new parameters
   * @param tR Target field relative heading 
   * @return Returns updated FieldRelativeCommand 
   */
  public FieldRelativeMoveCommand withHeading(double tR){
    m_targetRotation = tR;
    return this;

  }
 
}
