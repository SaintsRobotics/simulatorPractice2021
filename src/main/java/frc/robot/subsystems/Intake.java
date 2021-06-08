// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

public class Intake extends SubsystemBase {

  //private WPI_TalonSRX m_armController;  
  //private WPI_TalonSRX m_intakeController;  
  HardwareMap map = new HardwareMap();

  /** Creates a new Intake. */
  public Intake() {
  }

  public void intake() {
    map.intakeController.set(-0.5);
  }

  public void outtake() {
    map.intakeController.set(0.5);
  }

  public void arm(double direction) {
    map.armController.set(direction);
  }

}
