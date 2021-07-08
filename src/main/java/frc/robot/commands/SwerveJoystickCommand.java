package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants.SwerveConstants;

public class SwerveJoystickCommand extends CommandBase {
  private SwerveDrivetrain m_drivetrain;
  private XboxController m_controller;

  /**
   * Creates a new SwerveJoystickCommand.
   */
  public SwerveJoystickCommand(SwerveDrivetrain drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_controller = new XboxController(0);

  }

  @Override

  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // if dont apply deadzone, then relation between joystick/speed is linear and no
                          // deadzones, we need these
    double x = Utils.oddSquare(Utils.deadZones(m_controller.getY(Hand.kLeft), 0.2))
        * SwerveConstants.MAX_METERS_PER_SECOND; // apply functions to controller values to 1) check deadzone 2) apply
    // quadratic relation between controller/speed
    double y = Utils.oddSquare(Utils.deadZones(m_controller.getX(Hand.kLeft), 0.2))
        * SwerveConstants.MAX_METERS_PER_SECOND;
    double rot = Utils.oddSquare(Utils.deadZones(m_controller.getX(Hand.kRight), 0.2))
        * SwerveConstants.MAX_RADIANS_PER_SECOND;

    m_drivetrain.move(-x, -y, -rot, m_controller.getBumper(Hand.kRight));

  }

  @Override

  
  public void end(boolean interrupted) {
    m_drivetrain.move(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}