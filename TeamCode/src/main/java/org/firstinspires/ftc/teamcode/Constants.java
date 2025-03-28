package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // 5 is global, and the rest are in order
    public static double[] SWERVE_SERVO_ANGLE_OFFSET = {0, 0, 0, 0, 0};

    public static double[] SWERVE_STOP_FORMATION = {0, 0, 0, 0};

    public static final double kP = 0, kI = 0, kD = 0, kF = 0;

    public static final double ANALOG_MAX_VOLTAGE = 3.3;

    public static double PIDF_TOLERANCE_DEGREES = 2;

    public static final int SWERVE_MOTOR_RPM_CALCULATED_MAX = 5800;

    public static final int SWERVE_MOTOR_TPR = 28;

    public static final double SWERVE_MOTOR_TO_WHEEL_GEAR_RATIO = 6.74;

    public static final double WHEEL_RADIUS_MM = 62;

    public static final int WHEEL_CALCULATED_MAX_RPM = 861;
}
