// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  private WPI_VictorSPX m_kicker;
  private WPI_VictorSPX m_wheels;
  private SpeedControllerGroup m_feeder;
  private boolean m_isFeederOn;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    m_kicker = new WPI_VictorSPX(Constants.Feeder.KICKER_PORT);
    m_wheels = new WPI_VictorSPX(Constants.Feeder.WHEELS_PORT);
    m_wheels.setInverted(true);
    m_feeder = new SpeedControllerGroup(m_kicker, m_wheels);
    m_isFeederOn = false;
  }

  @Override
  public void periodic() {
    m_feeder.set(m_isFeederOn ? Constants.Feeder.FEEDER_SPEED : 0);

    SmartDashboard.putBoolean("Is Feeder On", m_isFeederOn);
    SmartDashboard.putNumber("Feeder Speed", m_feeder.get());
  }

  public void turnOnFeeder() {
    m_isFeederOn = true;
  }

  public void turnOffFeeder() {
    m_isFeederOn = false;
  }
}
