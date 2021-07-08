/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.MoveAction;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.HardwareMap;
import frc.robot.subsystems.Intake;


public class MoveArmCommand extends CommandBase {
  private XboxController m_controller;
  private Intake m_intakeSubsystem;
  /**
   * Creates a new MoveArmCommand.
   */
  public MoveArmCommand(XboxController controller, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller;
    m_intakeSubsystem = intake;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //joystickOutput will be between 1 and -1
    double joystickOutput = m_controller.getY(Hand.kLeft);
    m_intakeSubsystem.moveArm(-0.75*joystickOutput);
    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
