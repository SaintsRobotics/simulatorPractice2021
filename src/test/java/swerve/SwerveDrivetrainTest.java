import static org.junit.Assert.*;

import org.junit.*;
import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.SwerveDrivetrain;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class SwerveDrivetrainTest {
    private static final double DELTA = 1e-2;
    private SwerveDrivetrain drivetrain;

    @Before
    public void setup() {
        assert HAL.initialize(500, 0);
        drivetrain = new SwerveDrivetrain();

    }

    @After
    public void shutdown() throws Exception {
        drivetrain.close();
    }

    // @Test
    // public void startGyroisZero() {
    // assertEquals(drivetrain.getGyroAngle(), 0.0, DELTA);
    // }

    @Test
    public void unitTestTest2() {
        assertEquals(0.0, 0.0, DELTA);
    }
}
