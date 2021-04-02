/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.GoToPositionCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetOdometryCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();
  SwerveJoystickCommand swerveJoystickCommand = new SwerveJoystickCommand(swerveDrivetrain);
  ResetGyroCommand m_resetGyroCommand = new ResetGyroCommand(swerveDrivetrain);
  ResetOdometryCommand m_resetOdometryCommand = new ResetOdometryCommand(swerveDrivetrain);

  XboxController m_controller = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    swerveDrivetrain.setDefaultCommand(swerveJoystickCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton resetGyroButton = new JoystickButton(m_controller, 1);
    resetGyroButton.whenPressed(m_resetGyroCommand);

    JoystickButton resetOdometryButton = new JoystickButton(m_controller, 2);
    resetOdometryButton.whenPressed(m_resetOdometryCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // GoToPositionCommand firstPosition = new GoToPositionCommand(swerveDrivetrain,
    // 2.29, 1.52, 0);
    // GoToPositionCommand secondPosition = new
    // GoToPositionCommand(swerveDrivetrain, 4.57, 2.25, 2 * Math.PI);
    // GoToPositionCommand thirdPosition = new GoToPositionCommand(swerveDrivetrain,
    // 6.86, 1.52, 0);
    // GoToPositionCommand fourthPosition = new
    // GoToPositionCommand(swerveDrivetrain, 7.6, .76, 2 * Math.PI);
    // GoToPositionCommand fifthPosition = new GoToPositionCommand(swerveDrivetrain,
    // 8.4, 1.52, 0);
    // GoToPositionCommand sixthPosition = new GoToPositionCommand(swerveDrivetrain,
    // 7.6, 2.25, 2 * Math.PI);
    // GoToPositionCommand seventhPosition = new
    // GoToPositionCommand(swerveDrivetrain, 6.86, 1.52, 0);
    // GoToPositionCommand eigthPosition = new GoToPositionCommand(swerveDrivetrain,
    // 4.57, .76, 2 * Math.PI);
    // GoToPositionCommand ninethPositionCommand = new
    // GoToPositionCommand(swerveDrivetrain, 2.3, 1.5, 0);
    // GoToPositionCommand tenthPosition = new GoToPositionCommand(swerveDrivetrain,
    // 1, 2.5, 0);
    // return firstPosition.andThen(secondPosition,
    // thirdPosition).andThen(fourthPosition,fifthPosition).andThen(sixthPosition,
    // seventhPosition).andThen( eigthPosition, ninethPositionCommand,
    // tenthPosition);

  }

  public Command getTeleCommand() {
    return swerveJoystickCommand;
  }
}