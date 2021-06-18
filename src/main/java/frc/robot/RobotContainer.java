/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.MoveOneMeterCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetOdometryCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.FieldRelativeMoveCommand;
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
  private HardwareMap hardwareMap = new HardwareMap();
  public SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();
  private SwerveJoystickCommand swerveJoystickCommand = new SwerveJoystickCommand(swerveDrivetrain);
  public ResetGyroCommand m_resetGyroCommand = new ResetGyroCommand(swerveDrivetrain);
  public ResetOdometryCommand m_resetOdometryCommand = new ResetOdometryCommand(swerveDrivetrain);
  private String trajectoryJSON = "output/FirstOne.wpilib.json"; // change this to path following json
  private Trajectory trajectory = new Trajectory();
  private XboxController m_driverController = new XboxController(0);
  private XboxController m_operatorController = new XboxController(1);
  private Intake m_intakeSubsystem = new Intake(hardwareMap);
  private MoveArmCommand m_moveArmCommand = new MoveArmCommand(m_operatorController, m_intakeSubsystem);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    swerveDrivetrain.setDefaultCommand(swerveJoystickCommand);
    m_intakeSubsystem.setDefaultCommand(m_moveArmCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // resets gyro when A is pressed
    new JoystickButton(m_driverController, Button.kA.value).whenPressed(m_resetGyroCommand);

    // resets odometry when B is pressed
    new JoystickButton(m_driverController, Button.kB.value).whenPressed(m_resetOdometryCommand);

    // runs intake forwards while X is held
    new JoystickButton(m_operatorController, Button.kX.value).whenHeld(new IntakeCommand(m_intakeSubsystem));

    // runs the intake backwards while Y is held
    new JoystickButton(m_operatorController, Button.kY.value).whenHeld(new OuttakeCommand(m_intakeSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {


    // return new TurnToHeadingCommand(swerveDrivetrain).withRotation(30);
    // return new TurnToHeadingCommand(swerveDrivetrain).withHeading(Math.PI/2);
    // return pathFollowCommand().andThen(new StopCommand(swerveDrivetrain));
    //return new MoveOneMeterCommand(swerveDrivetrain);

    return new FieldRelativeMoveCommand(swerveDrivetrain).withX(3).withY(3).withHeading(Math.PI);

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
