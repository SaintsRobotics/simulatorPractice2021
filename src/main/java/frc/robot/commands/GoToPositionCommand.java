/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;

import javax.security.auth.x500.X500PrivateCredential;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants.SwerveConstants;

public class GoToPositionCommand extends CommandBase {
    protected SwerveDrivetrain m_drivetrain;
    private Pose2d m_currentPosition;
    protected PIDController m_xPID;
    protected PIDController m_yPID;
    protected PIDController m_rotationPID;
    private int m_counter;

    protected double m_targetX = Constants.defaultNull;
    protected double m_targetY = Constants.defaultNull;
    protected double m_targetRotation = Constants.defaultNull;

    /**
     * Creates a new GoToPositionCommand. Child classes pass in targetX, targetY,
     * targetR setpoints. If nothing is passed in, the current position is used as
     * the setpoint.
     */
    public GoToPositionCommand(SwerveDrivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;

        m_xPID = new PIDController(Constants.SwerveConstants.MAX_METERS_PER_SECOND, 0, 0);
        m_yPID = new PIDController(Constants.SwerveConstants.MAX_METERS_PER_SECOND, 0, 0);
        m_rotationPID = new PIDController(Math.PI * 6, 0, 0);

        m_xPID.setTolerance(0.05);
        m_yPID.setTolerance(0.05);
        m_rotationPID.setTolerance(Math.PI / 24);
        m_rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // Called when the command is initially scheduled.

    public void initialize() {
        if (m_targetX != Constants.defaultNull) {
            m_xPID.setSetpoint(m_targetX);
        } else {
            m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX());
        }

        if (m_targetY != Constants.defaultNull) {
            m_yPID.setSetpoint(m_targetY);
        } else {
            m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY());
        }

        if (m_targetRotation != Constants.defaultNull) {
            m_rotationPID.setSetpoint(m_targetRotation);
        } else {
            m_rotationPID.setSetpoint(m_drivetrain.getCurrentPosition().getRotation().getRadians());
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_currentPosition = m_drivetrain.getCurrentPosition();
        m_drivetrain.move(m_xPID.calculate(m_currentPosition.getX()), m_yPID.calculate(m_currentPosition.getY()),
                -m_rotationPID.calculate(m_currentPosition.getRotation().getRadians()), true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.move(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotationPID.atSetpoint()) {
            m_counter++;
        } else {
            m_counter = 0;
        }
        return m_counter > 10;
    }

}
