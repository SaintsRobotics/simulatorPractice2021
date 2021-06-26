// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HardwareMap;

/** The intake subsystem of the robot. */
public class Intake extends SubsystemBase {
  private VictorSPX intakeController;
  private VictorSPX armController;
  private double desiredSpeed;

  /**
   * Creates a new {@link Intake}.
   * 
   * @param hardwareMap The required class containing hardware.
   */
  public Intake(HardwareMap hardwareMap) {
    intakeController = hardwareMap.intakeController;
    armController = hardwareMap.armController;
    desiredSpeed = 0;
  }

  /** Runs the intake forwards. */
  public void intake() {
    desiredSpeed = Constants.intakeSpeed;
  }

  /** Runs the intake in reverse. */
  public void outtake() {
    desiredSpeed = -Constants.intakeSpeed;
  }

  /** Stops the intake. */
  public void stopIntake() {
    desiredSpeed = 0;
  }

  /**
   * Moves the arm with at a set speed.
   * 
   * @param speed The speed to move the arm.
   */
  public void moveArm(double speed) {
    armController.set(VictorSPXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake MotorSpeed", desiredSpeed);
    intakeController.set(VictorSPXControlMode.PercentOutput, desiredSpeed);
  }
}