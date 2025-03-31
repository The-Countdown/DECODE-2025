package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.Map;

/**
 * The `Status` class serves as a centralized repository for tracking the
 * operational state and positional context of various components. It
 * provides a unified interface for monitoring the current condition and
 * status of different parts of the robot, ensuring a cohesive understanding
 * of the robot's overall configuration at any given time.
 */
public class Status {
    /**
     * A map to store the status of each swerve servo.
     * The key is the integer, and the value is the ServoStatus enum.
     */
    public static Map<Integer, ServoStatus> swerveServoStatus = new HashMap<>();
    // Initialize the map in a static block
    static {
        swerveServoStatus.put(0, ServoStatus.TARGET_REACHED);
        swerveServoStatus.put(1, ServoStatus.TARGET_REACHED);
        swerveServoStatus.put(2, ServoStatus.TARGET_REACHED);
        swerveServoStatus.put(3, ServoStatus.TARGET_REACHED);
    }

    enum ServoStatus {
        TARGET_REACHED,
        MOVING
    }
}