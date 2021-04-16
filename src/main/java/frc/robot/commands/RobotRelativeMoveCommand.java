// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class RobotRelativeMoveCommand extends GoToPositionCommand {
    /** Creates a new RobotRelativeMoveCommand. */
    private double m_targetX;
    private double m_targetY;
    private double m_targetR;

    public RobotRelativeMoveCommand(SwerveDrivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        super(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_xPID.setSetpoint(m_drivetrain.getCurrentPosition().getX() + m_targetX);
        m_yPID.setSetpoint(m_drivetrain.getCurrentPosition().getY() + m_targetY);
        m_rotationPID.setSetpoint(m_targetR);
    }

    // for method chaining
    public RobotRelativeMoveCommand withX(double tX) { // e.g. creates command with some "x"
        m_targetX = tX;
        return this;
    }

    public RobotRelativeMoveCommand withY(double tY) {
        m_targetY = tY;
        return this;

    }

    public RobotRelativeMoveCommand withR(double tR) {
        m_targetR = tR;
        return this;

    }

}