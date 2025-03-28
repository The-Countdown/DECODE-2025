package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ThreadedIMU extends Thread {
    private Robot robot;
    public static double currentYaw = 0;

    public ThreadedIMU(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void run() {
    currentYaw = Robot.HardwareDevices.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        try {
            Thread.sleep(5);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
