/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AbsoluteEncoder;
import frc.robot.Constants.SwervePorts;
import frc.robot.Constants.SwerveConstants;

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
        private Gyro m_gyro;

        private SwerveDriveKinematics m_kinematics;

        // need pid to save headings/dynamic controls
        private PIDController m_rotationPID;

        /**
         * Creates a new SwerveDrivetrain.
         */
        public SwerveDrivetrain() {
                m_frontLeftDriveMotor = new CANSparkMax(SwervePorts.FRONT_LEFT_DRIVE_MOTOR_PORT, MotorType.kBrushless);
                m_frontRightDriveMotor = new CANSparkMax(SwervePorts.FRONT_RIGHT_DRIVE_MOTOR_PORT, MotorType.kBrushless);
                m_backLeftDriveMotor = new CANSparkMax(SwervePorts.BACK_LEFT_DRIVE_MOTOR_PORT, MotorType.kBrushless);
                m_backRightDriveMotor = new CANSparkMax(SwervePorts.BACK_RIGHT_DRIVE_MOTOR_PORT, MotorType.kBrushless);

                m_frontLeftTurningMotor = new CANSparkMax(SwervePorts.FRONT_LEFT_TURNING_MOTOR_PORT, MotorType.kBrushless);
                m_frontRightTurningMotor = new CANSparkMax(SwervePorts.FRONT_RIGHT_TURNING_MOTOR_PORT, MotorType.kBrushless);
                m_backLeftTurningMotor = new CANSparkMax(SwervePorts.BACK_LEFT_TURNING_MOTOR_PORT, MotorType.kBrushless);
                m_backRightTurningMotor = new CANSparkMax(SwervePorts.BACK_RIGHT_TURNING_MOTOR_PORT, MotorType.kBrushless);

                m_frontLeftDriveMotor.setInverted(true);
                m_backLeftDriveMotor.setInverted(true);

                m_frontLeftTurningEncoder = new AbsoluteEncoder(SwervePorts.FRONT_LEFT_TURNING_ENCODER_PORT, true, SwerveConstants.FRONT_LEFT_ROTATION_OFFSET);
                m_frontRightTurningEncoder = new AbsoluteEncoder(SwervePorts.FRONT_RIGHT_TURNING_ENCODER_PORT, true, SwerveConstants.FRONT_RIGHT_ROTATION_OFFSET);
                m_backLeftTurningEncoder = new AbsoluteEncoder(SwervePorts.BACK_LEFT_TURNING_ENCODER_PORT, true, SwerveConstants.BACK_LEFT_ROTATION_OFFSET);
                m_backRightTurningEncoder = new AbsoluteEncoder(SwervePorts.BACK_RIGHT_TURNING_ENCODER_PORT, true, SwerveConstants.BACK_RIGHT_ROTATION_OFFSET);

                // Robot is facing towards positive x direction
                m_frontLeftSwerveWheel = new SwerveWheel(m_frontLeftDriveMotor, m_frontLeftTurningMotor,
                                SwerveConstants.SWERVE_X, SwerveConstants.SWERVE_Y, m_frontLeftTurningEncoder);
                m_frontRightSwerveWheel = new SwerveWheel(m_frontRightDriveMotor, m_frontRightTurningMotor,
                                SwerveConstants.SWERVE_X, -SwerveConstants.SWERVE_Y, m_frontRightTurningEncoder);
                m_backLeftSwerveWheel = new SwerveWheel(m_backLeftDriveMotor, m_backLeftTurningMotor,
                                -SwerveConstants.SWERVE_X, SwerveConstants.SWERVE_Y, m_backLeftTurningEncoder);
                m_backRightSwerveWheel = new SwerveWheel(m_backRightDriveMotor, m_backRightTurningMotor,
                                -SwerveConstants.SWERVE_X, -SwerveConstants.SWERVE_Y, m_backRightTurningEncoder);

                m_kinematics = new SwerveDriveKinematics(
                                m_frontLeftSwerveWheel.getLocation(),
                                m_frontRightSwerveWheel.getLocation(),
                                m_backLeftSwerveWheel.getLocation(),
                                m_backRightSwerveWheel.getLocation());

                m_rotationPID = new PIDController(Math.toRadians((SwerveConstants.MAX_METERS_PER_SECOND / 180) * 5), 0, 0);
                m_rotationPID.enableContinuousInput(0, Math.PI * 2);
                m_rotationPID.setTolerance(1 / 36); // if off by a lil bit, then dont do anything (is in radians)

                m_gyro = new AHRS();
                
        }

        public void move(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative) {
                m_xSpeed = xSpeed;
                m_ySpeed = ySpeed;
                m_rotationSpeed = rotationSpeed;
                m_isFieldRelative = isFieldRelative;

                SmartDashboard.putNumber("X Speed", m_xSpeed);
                SmartDashboard.putNumber("Y Speed", m_ySpeed);
                SmartDashboard.putNumber("Rotation Speed", m_rotationSpeed);
        }

        @Override
        public void periodic() {
                Rotation2d gyroAngle = m_gyro.getRotation2d();

                ChassisSpeeds desiredSpeed;

                // convert to robot relative if in field relative
                if (this.m_isFieldRelative) {
                        desiredSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_rotationSpeed, gyroAngle);
                } else {
                        desiredSpeed = new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rotationSpeed);
                }

                SwerveModuleState[] desiredSwerveModuleStates = m_kinematics.toSwerveModuleStates(desiredSpeed);
                SwerveDriveKinematics.normalizeWheelSpeeds(desiredSwerveModuleStates, SwerveConstants.MAX_METERS_PER_SECOND);
                m_frontLeftSwerveWheel.setState(desiredSwerveModuleStates[0]);
                m_frontRightSwerveWheel.setState(desiredSwerveModuleStates[1]);
                m_backLeftSwerveWheel.setState(desiredSwerveModuleStates[2]);
                m_backRightSwerveWheel.setState(desiredSwerveModuleStates[3]);

                // after adjusting encoder code move to getAngle()
                SmartDashboard.putNumber("Front Left Turning Encoder", m_frontLeftTurningEncoder.getRadians());
                SmartDashboard.putNumber("Front Right Turning Encoder", m_frontRightTurningEncoder.getRadians());
                SmartDashboard.putNumber("Back Left Turning Encoder", m_backLeftTurningEncoder.getRadians());
                SmartDashboard.putNumber("Back Right Turning Encoder", m_backRightTurningEncoder.getRadians());
                SmartDashboard.putNumber("Gyro Heading", gyroAngle.getRadians());
        }
}