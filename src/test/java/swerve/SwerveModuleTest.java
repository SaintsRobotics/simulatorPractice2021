// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.Assert.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.AbsoluteEncoder;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwervePorts;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveWheel;

import org.junit.*;

/** Add your docs here. */
public class SwerveModuleTest {
    public static final double DELTA = 1e-2;
    private CANSparkMax motor;
    private CANSparkMax m_frontLeftDriveMotor;
    private CANSparkMax m_frontLeftTurningMotor;
    private SwerveWheel m_frontLeftSwerveWheel;
    private AbsoluteEncoder m_frontLeftTurningEncoder;

    @Before
    public void setup() {
        assert HAL.initialize(500, 0);
        motor = new CANSparkMax(50, MotorType.kBrushless);
        m_frontLeftDriveMotor = new CANSparkMax(SwervePorts.FRONT_LEFT_DRIVE_MOTOR_PORT, MotorType.kBrushless);
        m_frontLeftTurningMotor = new CANSparkMax(SwervePorts.FRONT_LEFT_TURNING_MOTOR_PORT, MotorType.kBrushless);
        m_frontLeftTurningEncoder = new AbsoluteEncoder(SwervePorts.FRONT_LEFT_TURNING_ENCODER_PORT, true,
                SwerveConstants.FRONT_LEFT_ROTATION_OFFSET);
        m_frontLeftSwerveWheel = new SwerveWheel(m_frontLeftDriveMotor, m_frontLeftTurningMotor,
                SwerveConstants.SWERVE_X, SwerveConstants.SWERVE_Y, m_frontLeftTurningEncoder);

    }

    @After
    public void shutdown() throws Exception {
        motor.close();
        m_frontLeftSwerveWheel.close();
    }

    @Test
    public void startingSwerveModuleStateIs() {

        assertEquals(m_frontLeftSwerveWheel.getState().compareTo(new SwerveModuleState(0.0, new Rotation2d(0))), 0.0,
                DELTA);
    }

    @Test
    public void unitTestTest() {
        assertEquals(0.0, 0.0, DELTA);
    }
    // @Test
    // public void startingGyroAngleisZero() {
    // assertEquals(drivetrain.getGyroAngle(), 0.0, DELTA);
    // }
}
