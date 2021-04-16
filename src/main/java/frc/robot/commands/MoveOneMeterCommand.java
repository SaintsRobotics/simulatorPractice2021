// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

public class MoveOneMeterCommand extends GoToPositionCommand {
    /** Creates a new MoveOneMeterCommand. */
    private double m_angle;

    public MoveOneMeterCommand(SwerveDrivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        super(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX() + Math.cos(m_angle));
        m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY() + Math.sin(m_angle));
        m_rotationPID.setSetpoint(m_drivetrain.getCurrentPosition().getRotation().getRadians());
    }

    public MoveOneMeterCommand withAngle(double angle) {
        m_angle = angle;
        return this;
    }

}
