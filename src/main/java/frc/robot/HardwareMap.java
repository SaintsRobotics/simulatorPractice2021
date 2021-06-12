package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;

public class HardwareMap {

    public WPI_TalonSRX intakeController;
    public WPI_TalonSRX armController;
    public XboxController operatorJoystick; 

    public HardwareMap() {
        //add a port for operatorJoystick
        intakeController = new WPI_TalonSRX(25);
        armController = new WPI_TalonSRX(24);
        operatorJoystick = new XboxController(1);
    }
}
