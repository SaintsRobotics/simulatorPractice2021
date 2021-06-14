// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXSimCollection;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

public class Intake extends SubsystemBase {

  // private WPI_TalonSRX m_armController;
  // private WPI_TalonSRX m_intakeController;
  private VictorSPX intakeController;
  private VictorSPX armController;
  private VictorSPXSimCollection intakeSim;
  private double desiredSpeed;

  // HardwareMap map = new HardwareMap();

  /** Creates a new Intake. */
  public Intake(HardwareMap hardwareMap) {
    intakeController = hardwareMap.intakeController;
    armController = hardwareMap.armController;
    intakeSim = intakeController.getSimCollection();
    desiredSpeed = 0;
  }

  public void intake() {
    desiredSpeed = 0.5;

  }

  public void outtake() {
    desiredSpeed = -0.5;
  }

  public void stopIntake() {
    desiredSpeed = 0;
  }

  public void arm(double speed) {
    armController.set(VictorSPXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake MotorSpeed", desiredSpeed);
    intakeController.set(VictorSPXControlMode.PercentOutput, desiredSpeed);
  }

}
