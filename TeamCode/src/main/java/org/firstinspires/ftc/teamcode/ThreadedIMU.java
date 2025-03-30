package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@SuppressWarnings("all")
public class ThreadedIMU extends Thread {
    private final Robot robot;
    public static double currentYaw = 0;

    /**
     * The IMU sensor requires approximately 3-5ms to complete a read operation.
     * Polling the IMU directly on the main thread would introduce significant latency.
     * To mitigate this, we utilize a dedicated thread for IMU data acquisition, allowing the main thread to continue without interruption.
     */
    public ThreadedIMU(Robot robot) {
        this.robot = robot;
        // Set the thread to be a daemon thread so that it will not prevent the program from exiting.
        setDaemon(true);
        setName("ThreadedIMU");
    }

    @Override
    public void run() {
        currentYaw = Robot.HardwareDevices.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        try {
            Thread.sleep(5);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
