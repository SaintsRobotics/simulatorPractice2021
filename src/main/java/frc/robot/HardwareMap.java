package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.XboxController;

public class HardwareMap {

    public VictorSPX intakeController;
    public VictorSPX armController;
    public XboxController operatorJoystick;

    public HardwareMap() {
        // add a port for operatorJoystick
        intakeController = new VictorSPX(25);
        armController = new VictorSPX(24);
        operatorJoystick = new XboxController(1);
    }
}
