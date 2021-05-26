// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class FeederCommand extends CommandBase {
  private ShooterSubsystem m_shooter;

  /** Creates a new FeederCommand. */
  public FeederCommand(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.turnOnFeeder();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.turnOffFeeder();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
