// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AbsoluteEncoder;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwervePorts;
import frc.robot.Robot;

/** The drive subsystem of the robot. */
public class SwerveDrivetrain extends SubsystemBase {
        private CANSparkMax m_frontLeftDriveMotor;
        private CANSparkMax m_frontRightDriveMotor;
        private CANSparkMax m_backLeftDriveMotor;
        private CANSparkMax m_backRightDriveMotor;

        private CANSparkMax m_frontLeftTurningMotor;
        private CANSparkMax m_frontRightTurningMotor;
        private CANSparkMax m_backLeftTurningMotor;
        private CANSparkMax m_backRightTurningMotor;

        private SwerveWheel m_frontLeftSwerveWheel;
        private SwerveWheel m_frontRightSwerveWheel;
        private SwerveWheel m_backLeftSwerveWheel;
        private SwerveWheel m_backRightSwerveWheel;

        private AbsoluteEncoder m_frontLeftTurningEncoder;
        private AbsoluteEncoder m_frontRightTurningEncoder;
        private AbsoluteEncoder m_backLeftTurningEncoder;
        private AbsoluteEncoder m_backRightTurningEncoder;

        private double m_xSpeed;
        private double m_ySpeed;
        private double m_rotationSpeed;
        private boolean m_isFieldRelative;
        private boolean isTurning = false;
        private ChassisSpeeds prevSpeed;
        private double currentHeading; // for simulated current gyro reading (yaw)
        private AHRS m_gyro;
        private SwerveDriveOdometry m_odometry;
        public SwerveDriveKinematics m_kinematics;
        private double time;
        private final Field2d m_field = new Field2d();
        // need pid to save headings/dynamic controls
        private PIDController m_rotationPID;

        private double m_gyroOffset = 0;

        private double m_desiredHeading;

        /**
         * Creates a new {@link SwerveDrivetrain}.
         */
        public SwerveDrivetrain() {
                SmartDashboard.putData("Field", m_field);
                m_frontLeftDriveMotor = new CANSparkMax(SwervePorts.FRONT_LEFT_DRIVE_MOTOR_PORT, MotorType.kBrushless);
                m_frontRightDriveMotor = new CANSparkMax(SwervePorts.FRONT_RIGHT_DRIVE_MOTOR_PORT,
                                MotorType.kBrushless);
                m_backLeftDriveMotor = new CANSparkMax(SwervePorts.BACK_LEFT_DRIVE_MOTOR_PORT, MotorType.kBrushless);
                m_backRightDriveMotor = new CANSparkMax(SwervePorts.BACK_RIGHT_DRIVE_MOTOR_PORT, MotorType.kBrushless);

                m_frontLeftTurningMotor = new CANSparkMax(SwervePorts.FRONT_LEFT_TURNING_MOTOR_PORT,
                                MotorType.kBrushless);
                m_frontLeftTurningMotor.setIdleMode(IdleMode.kCoast);
                m_frontRightTurningMotor = new CANSparkMax(SwervePorts.FRONT_RIGHT_TURNING_MOTOR_PORT,
                                MotorType.kBrushless);
                m_frontRightTurningMotor.setIdleMode(IdleMode.kCoast);
                m_backLeftTurningMotor = new CANSparkMax(SwervePorts.BACK_LEFT_TURNING_MOTOR_PORT,
                                MotorType.kBrushless);
                m_backLeftTurningMotor.setIdleMode(IdleMode.kCoast);
                m_backRightTurningMotor = new CANSparkMax(SwervePorts.BACK_RIGHT_TURNING_MOTOR_PORT,
                                MotorType.kBrushless);
                m_backRightTurningMotor.setIdleMode(IdleMode.kCoast);

                m_frontLeftTurningEncoder = new AbsoluteEncoder(SwervePorts.FRONT_LEFT_TURNING_ENCODER_PORT, true,
                                SwerveConstants.FRONT_LEFT_ROTATION_OFFSET);
                m_frontRightTurningEncoder = new AbsoluteEncoder(SwervePorts.FRONT_RIGHT_TURNING_ENCODER_PORT, true,
                                SwerveConstants.FRONT_RIGHT_ROTATION_OFFSET);
                m_backLeftTurningEncoder = new AbsoluteEncoder(SwervePorts.BACK_LEFT_TURNING_ENCODER_PORT, true,
                                SwerveConstants.BACK_LEFT_ROTATION_OFFSET);
                m_backRightTurningEncoder = new AbsoluteEncoder(SwervePorts.BACK_RIGHT_TURNING_ENCODER_PORT, true,
                                SwerveConstants.BACK_RIGHT_ROTATION_OFFSET);

                // Robot is facing towards positive x direction
                m_frontLeftSwerveWheel = new SwerveWheel("frontleft", m_frontLeftDriveMotor, m_frontLeftTurningMotor,
                                SwerveConstants.SWERVE_X, SwerveConstants.SWERVE_Y, m_frontLeftTurningEncoder);
                m_frontRightSwerveWheel = new SwerveWheel("frontright", m_frontRightDriveMotor,
                                m_frontRightTurningMotor, SwerveConstants.SWERVE_X, -SwerveConstants.SWERVE_Y,
                                m_frontRightTurningEncoder);
                m_backLeftSwerveWheel = new SwerveWheel("backleft", m_backLeftDriveMotor, m_backLeftTurningMotor,
                                -SwerveConstants.SWERVE_X, SwerveConstants.SWERVE_Y, m_backLeftTurningEncoder);
                m_backRightSwerveWheel = new SwerveWheel("backright", m_backRightDriveMotor, m_backRightTurningMotor,
                                -SwerveConstants.SWERVE_X, -SwerveConstants.SWERVE_Y, m_backRightTurningEncoder);

                m_kinematics = new SwerveDriveKinematics(m_frontLeftSwerveWheel.getLocation(),
                                m_frontRightSwerveWheel.getLocation(), m_backLeftSwerveWheel.getLocation(),
                                m_backRightSwerveWheel.getLocation());

                m_rotationPID = new PIDController(1, 0, 0);
                m_rotationPID.enableContinuousInput(-Math.PI, Math.PI);
                m_rotationPID.setTolerance(1 / 36); // if off by a lil bit, then dont do anything (is in radians)

                m_gyro = new AHRS();
                m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

                m_desiredHeading = 0;
        }

        /**
         * Returns the current {@link SwerveDriveKinematics}.
         * 
         * @return The current {@link SwerveDriveKinematics}.
         */
        public SwerveDriveKinematics getKinematics() {
                return m_kinematics;
        }

        // gyro should update in periodic
        // previous reading would be gyroAngle -> get in radians
        // want calculate how gyro changes based on rotation
        // angular speed: omegaRad/Sec multiply by sec, add on
        // manipulate in radians, convert to 2d again -> print radians

        /**
         * Method to drive the robot using joystick info.
         *
         * @param xSpeed          Speed of the robot in the x direction (forward).
         * @param ySpeed          Speed of the robot in the y direction (sideways).
         * @param rotationSpeed   Angular rate of the robot.
         * @param isFieldRelative Whether the provided x and y speeds are relative to
         *                        the field.
         */
        public void move(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative) {
                m_xSpeed = xSpeed;
                m_ySpeed = ySpeed;
                m_rotationSpeed = rotationSpeed;
                m_isFieldRelative = isFieldRelative;
                isTurning = (m_rotationSpeed != 0);

                // scales m_xSpeed and m_ySpeed such that the net speed is equal to
                // MAX_METERS_PER_SECOND (only if the net speed is above MAX_METERS_PER_SECOND)
                double m_netSpeed = Math.sqrt((m_xSpeed * m_xSpeed) + (m_ySpeed * m_ySpeed));
                if (m_netSpeed > SwerveConstants.MAX_METERS_PER_SECOND) {
                        // the scale factor will always be less than one
                        double m_scaleFactor = SwerveConstants.MAX_METERS_PER_SECOND / m_netSpeed;
                        m_xSpeed *= m_scaleFactor;
                        m_ySpeed *= m_scaleFactor;
                }

                if (m_rotationSpeed > SwerveConstants.MAX_RADIANS_PER_SECOND) {
                        m_rotationSpeed = SwerveConstants.MAX_RADIANS_PER_SECOND;
                }
                SmartDashboard.putNumber("X Speed", m_xSpeed);
                SmartDashboard.putNumber("Y Speed", m_ySpeed);
                SmartDashboard.putNumber("Rotation Speed", m_rotationSpeed);
        }

        /**
         * Sets the swerve ModuleStates.
         *
         * @param moduleStates The desired SwerveModule states.
         */
        public void move(SwerveModuleState... moduleStates) {
                ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(moduleStates);
                move(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
        }

        @Override
        public void periodic() {
                double gyroAngle = m_gyro.getAngle();
                if (time > 10) {
                        m_odometry.update(m_gyro.getRotation2d(), m_frontLeftSwerveWheel.getState(),
                                        m_frontRightSwerveWheel.getState(), m_backLeftSwerveWheel.getState(),
                                        m_backRightSwerveWheel.getState());
                        m_field.setRobotPose(m_odometry.getPoseMeters());
                }
                time++;
                ChassisSpeeds desiredSpeed;

                // heading correction code
                // if bot is turning, update setpoint
                // if bot is only translating, use the pid to correct heading
                if (isTurning) {
                        m_desiredHeading = Math.toRadians(gyroAngle);
                        m_rotationPID.setSetpoint(m_desiredHeading);
                } else if (m_xSpeed != 0 || m_ySpeed != 0) {
                        m_rotationSpeed = m_rotationPID.calculate(Math.toRadians(gyroAngle));
                }
                SmartDashboard.putNumber("rotation Speed", Math.toDegrees(m_rotationSpeed));

                // convert to robot relative if in field relative
                if (this.m_isFieldRelative) {
                        desiredSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_rotationSpeed,
                                        Rotation2d.fromDegrees(gyroAngle));
                } else {
                        desiredSpeed = new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rotationSpeed);
                }

                prevSpeed = desiredSpeed;

                SwerveModuleState[] desiredSwerveModuleStates = m_kinematics.toSwerveModuleStates(desiredSpeed);

                // If the robot is real, adds friction coefficient * max wheel speed to account
                // for friction
                if (Robot.isReal()) {
                        for (SwerveModuleState swerveModule : desiredSwerveModuleStates) {
                                swerveModule.speedMetersPerSecond += (SwerveConstants.TRANSLATIONAL_FRICTION
                                                * SwerveConstants.MAX_METERS_PER_SECOND);
                        }
                }

                SwerveDriveKinematics.normalizeWheelSpeeds(desiredSwerveModuleStates,
                                SwerveConstants.MAX_METERS_PER_SECOND);

                if (desiredSpeed.vxMetersPerSecond == 0 && desiredSpeed.vyMetersPerSecond == 0
                                && desiredSpeed.omegaRadiansPerSecond == 0) {
                        m_frontLeftSwerveWheel.setVelocity(0);
                        m_frontRightSwerveWheel.setVelocity(0);
                        m_backLeftSwerveWheel.setVelocity(0);
                        m_backRightSwerveWheel.setVelocity(0);
                } else {
                        m_frontLeftSwerveWheel.setState(desiredSwerveModuleStates[0]);
                        m_frontRightSwerveWheel.setState(desiredSwerveModuleStates[1]);
                        m_backLeftSwerveWheel.setState(desiredSwerveModuleStates[2]);
                        m_backRightSwerveWheel.setState(desiredSwerveModuleStates[3]);
                }

                // updates the gyro yaw value and prints it to the simulator
                double m_degreeRotationSpeed = Math.toDegrees(m_rotationSpeed);
                double m_degreesSinceLastTick = m_degreeRotationSpeed * Robot.kDefaultPeriod;

                printSimulatedGyro(m_gyro.getYaw() + m_degreesSinceLastTick + m_gyroOffset);

                SmartDashboard.putNumber("OdometryX", m_odometry.getPoseMeters().getX());
                SmartDashboard.putNumber("OdometryY", m_odometry.getPoseMeters().getY());
                SmartDashboard.putNumber("Odometryrot", m_odometry.getPoseMeters().getRotation().getDegrees());
                SmartDashboard.putNumber("Front Left Turning Encoder",
                                m_frontLeftTurningEncoder.getAngle().getRadians());
                SmartDashboard.putNumber("Front Right Turning Encoder",
                                m_frontRightTurningEncoder.getAngle().getRadians());
                SmartDashboard.putNumber("Back Left Turning Encoder", m_backLeftTurningEncoder.getAngle().getRadians());
                SmartDashboard.putNumber("Back Right Turning Encoder",
                                m_backRightTurningEncoder.getAngle().getRadians());
                SmartDashboard.putNumber("Gyro Heading", m_gyro.getYaw());
                SmartDashboard.putBoolean("Is it turning", isTurning);
                SmartDashboard.putNumber("Gyro angle in degrees", gyroAngle);
                SmartDashboard.putNumber("The desired angle", m_desiredHeading * (180 / Math.PI));
                SmartDashboard.putNumber("Angular Offset", m_rotationPID.getPositionError());
        }

        /**
         * Prints the estimated gyro value to the simulator.
         * 
         * @param printHeading The estimated gyro value.
         */
        public void printSimulatedGyro(double printHeading) {
                int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
                SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
                angle.set(printHeading);
        }

        /**
         * Returns the currently-estimated pose of the robot.
         *
         * @return The pose.
         */
        public Pose2d getCurrentPosition() {
                return m_odometry.getPoseMeters();
        }

        /** Zeroes the heading of the robot. */
        public void resetGyro() {
                if (Robot.isReal()) {
                        m_gyro.reset();
                } else {
                        m_gyroOffset = m_gyro.getYaw();
                }
        }

        /**
         * Zeroes the odometry.
         */
        public void resetOdometry() {
                m_odometry.resetPosition(new Pose2d(), new Rotation2d());
        }

        /**
         * Resets the odometry to the specified pose.
         *
         * @param position The pose to which to set the odometry.
         * @param angle    The angle to which to set the odometry.
         */
        public void resetOdometry(Pose2d position, Rotation2d angle) {
                m_odometry.resetPosition(position, angle);
        }
}