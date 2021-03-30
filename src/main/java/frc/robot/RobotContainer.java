/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToPositionCommand;
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    GoToPositionCommand P1 = new GoToPositionCommand(swerveDrivetrain, 0, 0, 0);
    GoToPositionCommand P2 = new GoToPositionCommand(swerveDrivetrain, 2.286, 0, 0);
    GoToPositionCommand P3 = new GoToPositionCommand(swerveDrivetrain, 2.286, 3.408, 0);
    GoToPositionCommand P4 = new GoToPositionCommand(swerveDrivetrain, 6.858, 3.408, 0);
    GoToPositionCommand P5 = new GoToPositionCommand(swerveDrivetrain, 6.858, 0, 0);
    GoToPositionCommand P6 = new GoToPositionCommand(swerveDrivetrain, 8.382, 0, 0);
    GoToPositionCommand P7 = new GoToPositionCommand(swerveDrivetrain, 8.382, 3.408, 0);
    GoToPositionCommand P8 = new GoToPositionCommand(swerveDrivetrain, 6.858, 3.408, 0);
    GoToPositionCommand P9 = new GoToPositionCommand(swerveDrivetrain, 6.858, 0, 0);
    GoToPositionCommand P10 = new GoToPositionCommand(swerveDrivetrain, 2.286, 0, 0);
    GoToPositionCommand P11 = new GoToPositionCommand(swerveDrivetrain, 2.286, 3.408, 0);
    GoToPositionCommand P12 = new GoToPositionCommand(swerveDrivetrain, 0, 3.408, 0);

    SequentialCommandGroup autonomousGroup = new SequentialCommandGroup(P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12);
    return autonomousGroup;
  }

  public Command getTeleCommand() {
    return swerveJoystickCommand;
  }
}