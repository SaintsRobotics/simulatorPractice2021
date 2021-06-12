package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;

public class HardwareMap {

    public TalonSRX intakeController;
    public TalonSRX armController;
    public XboxController operatorJoystick;

    public HardwareMap() {
        // add a port for operatorJoystick
        intakeController = new TalonSRX(25);
        armController = new TalonSRX(24);
        operatorJoystick = new XboxController(1);
    }
}
