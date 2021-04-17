// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Command to autonomously move the robot to a position relative to the robot
 */
public class MoveDirectionCommand extends GoToPositionCommand {
  /** Creates a new MoveDirectionCommand. */

  private double m_targetX;
  private double m_targetY;
  private double m_targetR;

  public MoveDirectionCommand(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX() + m_targetX);
    m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY() + m_targetY);
    m_rotationPID.setSetpoint(m_drivetrain.getCurrentPosition().getRotation().getRadians() + m_targetR);

  }

  /**
   * Updates the robot relative target X 
   * @param tX The desired displacement of the robot in the X direction
   * @return Updated command
   */
  public MoveDirectionCommand withX(double tX){ //e.g. creates command with some "x"
    m_targetX = tX;
    return this;
  }

  /**
   * Updates the robot relative target Y 
   * @param tY The desired displacement of the robot in the Y direction
   * @return Updated command
   */
  public MoveDirectionCommand withY(double tY){
    m_targetY = tY;
    return this;

  }

  /**
   * Updates the robot relative target rotation 
   * @param tR The desired angular displacement of the robot 
   * @return Updated command
   */
  public MoveDirectionCommand withR(double tR){
    m_targetR = tR;
    return this;
  }

}
