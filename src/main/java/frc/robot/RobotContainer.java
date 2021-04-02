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
    GoToPositionCommand firstPosition = new GoToPositionCommand(swerveDrivetrain, 0.762, 0.762, 0);
    GoToPositionCommand secondPosition = new GoToPositionCommand(swerveDrivetrain, 3.048, 2.286, Math.PI/4);
    GoToPositionCommand thirdPosition = new GoToPositionCommand(swerveDrivetrain, 4.572, 2.794, 0);
    GoToPositionCommand fourthPosition = new GoToPositionCommand(swerveDrivetrain, 5.715, 2.54, -Math.PI/4);
    GoToPositionCommand fifthPosition = new GoToPositionCommand(swerveDrivetrain, 6.7, 1.524, -Math.PI/4);
    GoToPositionCommand sixthPosition = new GoToPositionCommand(swerveDrivetrain, 7.62, 0.762, 0);
    GoToPositionCommand seventhPosition = new GoToPositionCommand(swerveDrivetrain, 8.763, 1.524, Math.PI/2);
    GoToPositionCommand eighthPosition = new GoToPositionCommand(swerveDrivetrain, 7.62, 2.54, -Math.PI);
    GoToPositionCommand ninthPosition = new GoToPositionCommand(swerveDrivetrain, 6.7, 1.524, -3*Math.PI/4);
    GoToPositionCommand tenthPosition = new GoToPositionCommand(swerveDrivetrain, 6.096, 0.762, -Math.PI);
    GoToPositionCommand elePosition = new GoToPositionCommand(swerveDrivetrain, 3.048, 0.635, 4*Math.PI/5);
    GoToPositionCommand twePosition = new GoToPositionCommand(swerveDrivetrain, 1.778, 2.032, 3*Math.PI/4);
    GoToPositionCommand lastPosition = new GoToPositionCommand(swerveDrivetrain, 0.762, 2.286, -Math.PI);
    return new SequentialCommandGroup(firstPosition, secondPosition, thirdPosition, fourthPosition, fifthPosition, sixthPosition, seventhPosition, eighthPosition
    , ninthPosition, tenthPosition, elePosition, twePosition, lastPosition);
    
  }

  public Command getTeleCommand() {
    return swerveJoystickCommand;
  }
}