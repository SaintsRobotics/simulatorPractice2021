/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
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
        m_turningPIDController.enableContinuousInput(0, 2 * Math.PI);
        m_turningEncoder = encoder;
    }

    public void setState(SwerveModuleState state) {
        m_driveMotor.set(state.speedMetersPerSecond / SwerveConstants.MAX_METERS_PER_SECOND);
        m_turningPIDController.setSetpoint(state.angle.getRadians());

        // desired turn voltage
        double percentVoltage = m_turningPIDController.calculate(m_turningEncoder.getRadians());
        if (Robot.isSimulation()) {
            m_turningEncoder.sendVoltage(percentVoltage);
        }
        m_turningMotor.set(percentVoltage);
    }

    public Translation2d getLocation() {
        return m_location;
    }

}