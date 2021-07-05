// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/** Controls the drivetrain with an Xbox controller. */
public class SwerveJoystickCommand extends CommandBase {
	private SwerveDrivetrain m_drivetrain;
	private XboxController m_controller;

	/**
	 * Creates a new {@link SwerveJoystickCommand}.
	 * 
	 * @param drivetrain The {@link SwerveDrivetrain} subsystem to use.
	 */
	public SwerveJoystickCommand(SwerveDrivetrain drivetrain) {
		addRequirements(drivetrain);
		m_drivetrain = drivetrain;
		m_controller = new XboxController(0);
	}

	@Override
	public void execute() {
		// apply functions to controller values to 1) check deadzone 2) apply quadratic
		// relation between controller/speed
		double x = Utils.oddSquare(Utils.deadZones(-m_controller.getY(Hand.kLeft), 0.2))
				* DriveConstants.MAX_SPEED_METERS_PER_SECOND * 0.2;
		double y = Utils.oddSquare(Utils.deadZones(m_controller.getX(Hand.kLeft), 0.2))
				* DriveConstants.MAX_SPEED_METERS_PER_SECOND * 0.2;
		double rot = Utils.oddSquare(Utils.deadZones(-m_controller.getX(Hand.kRight), 0.2))
				* ModuleConstants.MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND * 0.2;

		m_drivetrain.drive(x, y, rot, m_controller.getBumper(Hand.kRight));
	}

	@Override
	public void end(boolean interrupted) {
		m_drivetrain.drive(0, 0, 0, false);
	}
}