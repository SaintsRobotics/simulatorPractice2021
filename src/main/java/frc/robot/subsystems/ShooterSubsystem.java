// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
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

  private WPI_VictorSPX m_kicker;
  private WPI_VictorSPX m_wheels;
  private SpeedControllerGroup m_feeder;
  private boolean m_isFeederOn;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_leftShooter = new CANSparkMax(Constants.Shooter.LEFT_SHOOTER_PORT, MotorType.kBrushless);
    m_rightShooter = new CANSparkMax(Constants.Shooter.LEFT_SHOOTER_PORT, MotorType.kBrushless);
    m_leftShooter.setInverted(true);
    m_shooter = new SpeedControllerGroup(m_leftShooter, m_rightShooter);
    m_isShooterOn = false;

    m_kicker = new WPI_VictorSPX(Constants.Shooter.KICKER_PORT);
    m_wheels = new WPI_VictorSPX(Constants.Shooter.WHEELS_PORT);
    m_wheels.setInverted(true);
    m_feeder = new SpeedControllerGroup(m_kicker, m_wheels);
    m_isFeederOn = false;
  }

  @Override
  public void periodic() {
    m_shooter.set(m_isShooterOn ? Constants.Shooter.SHOOTER_SPEED : 0);
    m_feeder.set(m_isFeederOn ? Constants.Shooter.FEEDER_SPEED : 0);

    SmartDashboard.putBoolean("Is Shooter On", m_isShooterOn);
    SmartDashboard.putNumber("Shooter Speed", m_shooter.get());
    SmartDashboard.putBoolean("Is Feeder On", m_isShooterOn);
    SmartDashboard.putNumber("Feeder Speed", m_feeder.get());
  }

  public void turnOnShooter() {
    m_isShooterOn = true;
  }

  public void turnOffShooter() {
    m_isShooterOn = false;
  }

  public void turnOnFeeder() {
    m_isFeederOn = true;
  }

  public void turnOffFeeder() {
    m_isFeederOn = false;
  }
}
