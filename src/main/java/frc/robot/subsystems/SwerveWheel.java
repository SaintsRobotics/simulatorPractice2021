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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private String m_name;

    public SwerveWheel(String name, CANSparkMax driveMotor, CANSparkMax turningMotor, double x, double y, AbsoluteEncoder encoder) {
        m_driveMotor = driveMotor;
        m_turningMotor = turningMotor;
        m_location = new Translation2d(x, y);
        m_turningPIDController = new PIDController(0.3, 0, 0);
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_turningPIDController.setTolerance(Math.PI/18);
        m_turningEncoder = encoder;
        m_name = name;
        m_state = new SwerveModuleState();
    }

    public void setState(SwerveModuleState state) {

        // literally just smart inversion
        //state = SwerveModuleState.optimize(state, m_turningEncoder.getAngle());

        m_turningPIDController.setSetpoint(state.angle.getRadians());

        // desired turn voltage to send to turning motor, range: [-1, 1]
        double percentVoltage = m_turningPIDController.calculate(m_turningEncoder.getAngle().getRadians());

        if (Robot.isSimulation()) {
            m_turningEncoder.sendVoltage(percentVoltage);
        }

        m_driveMotor.set(state.speedMetersPerSecond / SwerveConstants.MAX_METERS_PER_SECOND);
        m_turningMotor.set(percentVoltage);
        m_state = new SwerveModuleState(state.speedMetersPerSecond, m_turningEncoder.getAngle());

        SmartDashboard.putNumber("Wheel " + m_name + " Voltage", percentVoltage);
        SmartDashboard.putNumber("Wheel " + m_name + " Current Position", m_turningEncoder.getAngle().getDegrees());
        SmartDashboard.putNumber("Wheel " + m_name + " Desired Position", state.angle.getDegrees());
        SmartDashboard.putNumber("Wheel " + m_name + " Error", m_turningPIDController.getPositionError());


        

    }

    public Translation2d getLocation() {
        return m_location;
    }

    public SwerveModuleState getState() {
        return m_state;
    }

    public void setVelocity(double speed) {
        m_state.speedMetersPerSecond = speed;
        m_driveMotor.set(speed / SwerveConstants.MAX_METERS_PER_SECOND);
        m_turningMotor.set(0);
    }

}