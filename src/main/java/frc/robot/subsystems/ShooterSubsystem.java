// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_leftShooter;
  private CANSparkMax m_rightShooter;
  private SpeedControllerGroup m_shooter;
  private boolean m_isShooterOn;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_leftShooter = new CANSparkMax(Constants.Shooter.LEFT_SHOOTER_PORT, MotorType.kBrushless);
    m_rightShooter = new CANSparkMax(Constants.Shooter.LEFT_SHOOTER_PORT, MotorType.kBrushless);
    m_leftShooter.setInverted(true);
    m_shooter = new SpeedControllerGroup(m_leftShooter, m_rightShooter);
    m_isShooterOn = false;
  }

  @Override
  public void periodic() {
    m_shooter.set(m_isShooterOn ? Constants.Shooter.SHOOTER_SPEED : 0);

    SmartDashboard.putBoolean("Is Shooter On", m_isShooterOn);
    SmartDashboard.putNumber("Shooter Speed", m_shooter.get());
  }

  public void turnOnShooter() {
    m_isShooterOn = true;
  }

  public void turnOffShooter() {
    m_isShooterOn = false;
  }
}
