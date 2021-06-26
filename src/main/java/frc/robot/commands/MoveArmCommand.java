// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveArmCommand extends CommandBase {
  private XboxController m_controller;
  private Intake m_intakeSubsystem;

  /**
   * Creates a new {@link MoveArmCommand}.
   * 
   * @param controller The XboxController to get inputs from.
   * @param intake     The {@link Intake} subsystem to use.
   */
  public MoveArmCommand(XboxController controller, Intake intake) {
    m_controller = controller;
    m_intakeSubsystem = intake;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void execute() {
    // joystickOutput will be between 1 and -1
    double joystickOutput = m_controller.getY(Hand.kLeft);
    m_intakeSubsystem.moveArm(-joystickOutput);
  }
}