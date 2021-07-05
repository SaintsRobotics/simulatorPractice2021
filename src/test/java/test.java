
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import static org.junit.Assert.*;
import org.junit.*;
/** Add your docs here. */
import edu.wpi.first.hal.HAL;

public class test {
    @Before
    public void setup() {
        assert HAL.initialize(500, 0);
    }

    @Test
    public void test() {
        assertEquals(1, 1);
    }
}
