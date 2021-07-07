
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import static org.junit.Assert.*;
import org.junit.*;
/** Add your docs here. */
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.HardwareMap;
import frc.robot.commands.MoveDirectionCommand;
import frc.robot.commands.MoveOneMeterCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class test {
    private HardwareMap hardwareMap;
    private SwerveDrivetrain drivetrain;

    @Before
    public void setup() {
        assert HAL.initialize(500, 0);
        hardwareMap = new HardwareMap();
        drivetrain = new SwerveDrivetrain(hardwareMap);

    }

    @After
    public void end() {
        hardwareMap.close();
        drivetrain.close();
    }

    @Test
    public void atesting2() {
        assertEquals(2, 2);
    }

    @Test
    public void initialPosition() {
        assertEquals(0, drivetrain.getPose().getX(), 0.1);
    }

    @Test
    public void moveOneMeter() {
        MoveOneMeterCommand moveOneMeterCommand = new MoveOneMeterCommand(drivetrain);
        moveOneMeterCommand.schedule();
        int i = 0;
        while (i < 1000) {
            CommandScheduler.getInstance().run();
            i++;
        }
        System.out.println(drivetrain.getPose().toString());
        assertEquals(0, drivetrain.getPose().getX(), 0.1);

    }
}
