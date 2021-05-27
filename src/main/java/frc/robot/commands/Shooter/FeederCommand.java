// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCommand extends CommandBase {
  private FeederSubsystem m_feeder;

  /** Creates a new FeederCommand. */
  public FeederCommand(FeederSubsystem feeder) {
    m_feeder = feeder;
    addRequirements(m_feeder);
  }

  @Override
  public void initialize() {
    m_feeder.turnOnFeeder();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_feeder.turnOffFeeder();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
