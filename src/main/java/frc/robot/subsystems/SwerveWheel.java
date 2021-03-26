/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AbsoluteEncoder;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;

/**
 * Add your docs here.
 */
public class SwerveWheel {
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turningMotor;
    private Translation2d m_location;
    private PIDController m_turningPIDController;
    private AbsoluteEncoder m_turningEncoder;
    private SwerveModuleState m_state;

    public SwerveWheel(CANSparkMax driveMotor, CANSparkMax turningMotor, double x, double y, AbsoluteEncoder encoder) {
        m_driveMotor = driveMotor;
        m_turningMotor = turningMotor;
        m_location = new Translation2d(x, y);
        m_turningPIDController = new PIDController(.3, 0, 0);
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_turningEncoder = encoder;
    }

    public void setState(SwerveModuleState state) {
        state = smartInversion(state);

        m_turningPIDController.setSetpoint(state.angle.getRadians());

        // desired turn voltage to send to turning motor, range: [-1, 1]
        double percentVoltage = m_turningPIDController.calculate(m_turningEncoder.getRadians());

        if (Robot.isSimulation()) {
            m_turningEncoder.sendVoltage(percentVoltage);
        }

        m_driveMotor.set(state.speedMetersPerSecond / SwerveConstants.MAX_METERS_PER_SECOND);
        m_turningMotor.set(percentVoltage);
        m_state = new SwerveModuleState(state.speedMetersPerSecond, new Rotation2d(m_turningEncoder.getRadians()));
        // m_state is the ACTUAL CURRENT state of the wheel, not the state given to us
        // by SwerveSubsystem.
        // The difference is that m_state had smart inversion math done to it, and it's
        // the current angle of the wheel, not the PID angle setpoint.
    }

    public Translation2d getLocation() {
        return m_location;
    }

    public SwerveModuleState getState() {
        return m_state;
    }

    public SwerveModuleState smartInversion(SwerveModuleState state) {
        double targetVeloicty = state.speedMetersPerSecond;
        double targetAngle = state.angle.getRadians();
        SmartDashboard.putNumber("target angle 1", targetAngle);

        double currentAngle = m_turningEncoder.getRadians();
        double angleDiff = Math.abs(targetAngle - currentAngle);

        if (angleDiff > Math.PI) {
            angleDiff = 2 * Math.PI - angleDiff;
        }
        // this is literally the same thing the pid does; we're just trynna keep up

        if (angleDiff > (Math.PI / 2)) {
            targetAngle += Math.PI; // adjusts desired state angle of wheel
            targetAngle = (((targetAngle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI)); // another mod-add-mod cuz
                                                                                             // why not
            targetVeloicty *= -1; // inverts wheel speed
        }

        SmartDashboard.putNumber("Angle dif", angleDiff);
        SmartDashboard.putNumber("target angle 2", targetAngle);

        return new SwerveModuleState(targetVeloicty, new Rotation2d(targetAngle));
    }
}