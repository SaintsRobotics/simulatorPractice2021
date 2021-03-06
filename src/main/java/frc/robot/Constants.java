// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final double defaultNull = Double.MIN_VALUE;
	public static final double intakeSpeed = 0.5;

	public static final class DriveConstants {
		public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 8;
		public static final int REAR_LEFT_DRIVE_MOTOR_PORT = 2;
		public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 4;
		public static final int REAR_RIGHT_DRIVE_MOTOR_PORT = 6;

		public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 1;
		public static final int REAR_LEFT_TURNING_MOTOR_PORT = 3;
		public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 5;
		public static final int REAR_RIGHT_TURNING_MOTOR_PORT = 7;

		public static final int FRONT_LEFT_TURNING_ENCODER_PORT = 0;
		public static final int REAR_LEFT_TURNING_ENCODER_PORT = 3;
		public static final int FRONT_RIGHT_TURNING_ENCODER_PORT = 1;
		public static final int REAR_RIGHT_TURNING_ENCODER_PORT = 2;

		public static final double MAX_SPEED_METERS_PER_SECOND = 3.627;
	}

	public static final class ModuleConstants {
		public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 8.76;

		/** Distance between centers of right and left wheels on robot. */
		public static final double TRACK_WIDTH = .67;

		/** Distance between front and back wheels on robot. */
		public static final double WHEEL_BASE = .25 * 2;

		public static final double TRANSLATIONAL_FRICTION = 0.0205;

		public static final double FRONT_LEFT_ROTATION_OFFSET = 2.75 - (Math.PI / 5);
		public static final double REAR_LEFT_ROTATION_OFFSET = -6.091199;
		public static final double FRONT_RIGHT_ROTATION_OFFSET = 2.573;
		public static final double REAR_RIGHT_ROTATION_OFFSET = 3.9;
	}
}