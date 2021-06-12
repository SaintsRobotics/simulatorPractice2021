/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.FieldRelativeMoveCommand;
import frc.robot.commands.GoToPositionCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveDirectionCommand;
import frc.robot.commands.MoveOneMeterCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetOdometryCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TurnToHeadingCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...the ones button
  // bound/fundamental
  public SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();
  private SwerveJoystickCommand swerveJoystickCommand = new SwerveJoystickCommand(swerveDrivetrain);
  public ResetGyroCommand m_resetGyroCommand = new ResetGyroCommand(swerveDrivetrain);
  public ResetOdometryCommand m_resetOdometryCommand = new ResetOdometryCommand(swerveDrivetrain);
  private String trajectoryJSON = "output/FirstOne.wpilib.json"; //change this to path following json
  private Trajectory trajectory = new Trajectory();
  private XboxController m_controller = new XboxController(0);
  private XboxController m_operatorController = new XboxController(1);
  private Intake m_intakeSubsystem = new Intake();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    swerveDrivetrain.setDefaultCommand(swerveJoystickCommand);
    m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem));
    
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

    JoystickButton runIntakeXButton = new JoystickButton(m_controller, 3);
    runIntakeXButton.whileHeld(new IntakeCommand(m_intakeSubsystem)); //X Button

    JoystickButton runOuttakeYButton = new JoystickButton(m_operatorController, 4);
    runOuttakeYButton.whileHeld(new OuttakeCommand(m_intakeSubsystem)); //Y Button


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //return new TurnToHeadingCommand(swerveDrivetrain).withRotation(30);
    //return new TurnToHeadingCommand(swerveDrivetrain).withHeading(Math.PI/2);
    //return pathFollowCommand().andThen(new StopCommand(swerveDrivetrain));
    return new MoveOneMeterCommand(swerveDrivetrain);    
  }

  public Command getTeleCommand() {
    return swerveJoystickCommand;
  }

  public Command getTestCommand() {
    // return new MoveOneMeterCommand(swerveDrivetrain).andThen(new
    // MoveOneMeterCommand(swerveDrivetrain));
    return null;
  }

  public Command pathFollowCommand() {

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    PIDController xPID = new PIDController(Constants.SwerveConstants.MAX_METERS_PER_SECOND, 0, 0);
    PIDController yPID = new PIDController(Constants.SwerveConstants.MAX_METERS_PER_SECOND, 0, 0);
    ProfiledPIDController rotPID = new ProfiledPIDController(-Math.PI * 6, 0.0, 0.0,
        new TrapezoidProfile.Constraints(Constants.SwerveConstants.MAX_RADIANS_PER_SECOND, 2.6));
    xPID.setTolerance(.05);
    yPID.setTolerance(0.05);
    rotPID.setTolerance(Math.PI / 24);
    rotPID.enableContinuousInput(-Math.PI, Math.PI);
    return new SwerveControllerCommand(trajectory, swerveDrivetrain::getCurrentPosition,
        swerveDrivetrain.getKinematics(), xPID, yPID, rotPID, swerveDrivetrain::move, swerveDrivetrain);

  }

}