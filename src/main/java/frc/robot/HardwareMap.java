// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.XboxController;

public class HardwareMap {
	public VictorSPX intakeController;
	public VictorSPX armController;
	public XboxController operatorJoystick;

	public HardwareMap() {
		// add a port for operatorJoystick
		intakeController = new VictorSPX(25);
		armController = new VictorSPX(24);
		operatorJoystick = new XboxController(1);
	}
}