// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

public class Intake extends SubsystemBase {

  //private WPI_TalonSRX m_armController;  
  //private WPI_TalonSRX m_intakeController;  
  private WPI_TalonSRX intakeController;
  private WPI_TalonSRX armController;
  //HardwareMap map = new HardwareMap();

  /** Creates a new Intake. */
  public Intake() {
    intakeController = new WPI_TalonSRX(25);
    armController = new WPI_TalonSRX(24);
  }

  public void intake() {
    intakeController.set(0.5);
  }

  public void outtake() {
    intakeController.set(-0.5);
  }

  public void stopIntake() {
    intakeController.set(0);
  }

  public void arm(double speed) {
    armController.set(speed);
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Intake Motor", intakeController.get());
  }

}
