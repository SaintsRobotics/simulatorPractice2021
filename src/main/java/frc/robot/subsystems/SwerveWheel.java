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
    private int inversionConstant;

    public SwerveWheel(CANSparkMax driveMotor, CANSparkMax turningMotor, double x, double y, AbsoluteEncoder encoder) {
        m_driveMotor = driveMotor;
        m_turningMotor = turningMotor;
        m_location = new Translation2d(x, y);
        m_turningPIDController = new PIDController(.3, 0, 0);
        m_turningPIDController.enableContinuousInput(0, 2 * Math.PI);
        m_turningEncoder = encoder;
    }

    public void setState(SwerveModuleState state) {
        inversionConstant = 1;

        m_turningPIDController.setSetpoint(smartInversion(state.angle.getRadians()));

        // desired turn voltage to send to turning motor, range: [-1, 1]
        double percentVoltage = m_turningPIDController.calculate(m_turningEncoder.getRadians());
        
        if (Robot.isSimulation()) {
            m_turningEncoder.sendVoltage(percentVoltage);
        }

        m_driveMotor.set(inversionConstant * state.speedMetersPerSecond / SwerveConstants.MAX_METERS_PER_SECOND);

        m_turningMotor.set(percentVoltage);
        m_state = new SwerveModuleState(state.speedMetersPerSecond, new Rotation2d(m_turningEncoder.getRadians()));
    }

    public Translation2d getLocation() {
        return m_location;
    }

    public SwerveModuleState getState() {
        return m_state;
    }

    public double smartInversion(double goal) {
        double current = m_turningEncoder.getRadians();
        if (Math.abs(goal-current) > (Math.PI/2)) {
            goal += Math.PI;
            inversionConstant = -1;
        }
        return goal;
    }

}