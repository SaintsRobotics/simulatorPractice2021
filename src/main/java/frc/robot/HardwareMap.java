package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;

public class HardwareMap {

    public VictorSPX intakeController;
    public VictorSPX armController;
    public XboxController operatorJoystick;
    public class ShooterHardware{
        public CANSparkMax leftShooter = new CANSparkMax(16, MotorType.kBrushless );
        public CANSparkMax rightShooter = new CANSparkMax(17, MotorType.kBrushless);
        public SpeedControllerGroup shooter = new SpeedControllerGroup(leftShooter, rightShooter);
        public ShooterHardware(){
            rightShooter.setInverted(true);
        }
    };

    public HardwareMap() {
        // add a port for operatorJoystick
        intakeController = new VictorSPX(25);
        armController = new VictorSPX(24);
        operatorJoystick = new XboxController(1);
    }
}
