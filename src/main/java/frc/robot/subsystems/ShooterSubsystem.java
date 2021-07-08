// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;
import frc.robot.HardwareMap.ShooterHardware;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_leftShooter;
  private CANSparkMax m_rightShooter;
  private SpeedControllerGroup m_shooter;
  private double m_targetSpeed;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(ShooterHardware shooter) {
    m_leftShooter = shooter.leftShooter;
    m_rightShooter = shooter.rightShooter;
  
    m_shooter = shooter.shooter;
  }
  public void setShooter(double speed){
    this.m_targetSpeed = speed;
  }

  @Override
  public void periodic() {
    m_shooter.set(m_targetSpeed);
    // This method will be called once per scheduler run

  }
}
