// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SwerveModule;

public class HardwareMap implements AutoCloseable {
	public VictorSPX intakeController;
	public VictorSPX armController;
	public XboxController operatorJoystick;
	public CANSparkMax frontLeftDriveMotor;
	public CANSparkMax frontRightDriveMotor;
	public CANSparkMax backLeftDriveMotor;
	public CANSparkMax backRightDriveMotor;
	public CANSparkMax frontLeftTurningMotor;
	public CANSparkMax frontRightTurningMotor;
	public CANSparkMax backLeftTurningMotor;
	public CANSparkMax backRightTurningMotor;
	public AbsoluteEncoder frontLeftTurningEncoder;
	public AbsoluteEncoder frontRightTurningEncoder;
	public AbsoluteEncoder backLeftTurningEncoder;
	public AbsoluteEncoder backRightTurningEncoder;
	public SwerveModule frontLeftSwerveModule;
	public SwerveModule frontRightSwerveModule;
	public SwerveModule backLeftSwerveModule;
	public SwerveModule backRightSwerveModule;

	public HardwareMap() {
		frontLeftDriveMotor = new CANSparkMax(DriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT, MotorType.kBrushless);
		frontRightDriveMotor = new CANSparkMax(DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT, MotorType.kBrushless);
		backLeftDriveMotor = new CANSparkMax(DriveConstants.REAR_LEFT_DRIVE_MOTOR_PORT, MotorType.kBrushless);
		backRightDriveMotor = new CANSparkMax(DriveConstants.REAR_RIGHT_DRIVE_MOTOR_PORT, MotorType.kBrushless);

		frontLeftTurningMotor = new CANSparkMax(DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT, MotorType.kBrushless);
		frontLeftTurningMotor.setIdleMode(IdleMode.kCoast);
		frontRightTurningMotor = new CANSparkMax(DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT, MotorType.kBrushless);
		frontRightTurningMotor.setIdleMode(IdleMode.kCoast);
		backLeftTurningMotor = new CANSparkMax(DriveConstants.REAR_LEFT_TURNING_MOTOR_PORT, MotorType.kBrushless);
		backLeftTurningMotor.setIdleMode(IdleMode.kCoast);
		backRightTurningMotor = new CANSparkMax(DriveConstants.REAR_RIGHT_TURNING_MOTOR_PORT, MotorType.kBrushless);
		backRightTurningMotor.setIdleMode(IdleMode.kCoast);

		frontLeftTurningEncoder = new AbsoluteEncoder(DriveConstants.FRONT_LEFT_TURNING_ENCODER_PORT, true,
				ModuleConstants.FRONT_LEFT_ROTATION_OFFSET);
		frontRightTurningEncoder = new AbsoluteEncoder(DriveConstants.FRONT_RIGHT_TURNING_ENCODER_PORT, true,
				ModuleConstants.FRONT_RIGHT_ROTATION_OFFSET);
		backLeftTurningEncoder = new AbsoluteEncoder(DriveConstants.REAR_LEFT_TURNING_ENCODER_PORT, true,
				ModuleConstants.REAR_LEFT_ROTATION_OFFSET);
		backRightTurningEncoder = new AbsoluteEncoder(DriveConstants.REAR_RIGHT_TURNING_ENCODER_PORT, true,
				ModuleConstants.REAR_RIGHT_ROTATION_OFFSET);

		// Robot is facing towards positive x direction
		frontLeftSwerveModule = new SwerveModule("Front Left Swerve Module", frontLeftDriveMotor, frontLeftTurningMotor,
				ModuleConstants.TRACK_WIDTH / 2, ModuleConstants.WHEEL_BASE, frontLeftTurningEncoder);
		frontRightSwerveModule = new SwerveModule("Front Right Swerve Module", frontRightDriveMotor,
				frontRightTurningMotor, ModuleConstants.TRACK_WIDTH, -ModuleConstants.WHEEL_BASE,
				frontRightTurningEncoder);
		backLeftSwerveModule = new SwerveModule("Back Left Swerve Module", backLeftDriveMotor, backLeftTurningMotor,
				-ModuleConstants.TRACK_WIDTH, ModuleConstants.WHEEL_BASE, backLeftTurningEncoder);
		backRightSwerveModule = new SwerveModule("Back Right Swerve Module", backRightDriveMotor, backRightTurningMotor,
				-ModuleConstants.TRACK_WIDTH, -ModuleConstants.WHEEL_BASE, backRightTurningEncoder);

		// add a port for operatorJoystick
		intakeController = new VictorSPX(25);
		armController = new VictorSPX(24);
		operatorJoystick = new XboxController(1);
	}

	@Override
	public void close() {
		// TODO Auto-generated method stub
		frontLeftDriveMotor.close();
		frontRightDriveMotor.close();
		backLeftDriveMotor.close();
		backRightDriveMotor.close();

		frontLeftTurningMotor.close();
		frontRightTurningMotor.close();
		backLeftTurningMotor.close();
		backRightTurningMotor.close();

		frontLeftTurningEncoder.close();
		frontRightTurningEncoder.close();
		backLeftTurningEncoder.close();
		backRightTurningEncoder.close();
	}

}