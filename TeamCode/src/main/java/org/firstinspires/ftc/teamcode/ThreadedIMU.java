package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ThreadedIMU extends Thread {
    private final RobotManager robotManager;
    public static double currentYaw = 0;

    /**
     * The IMU sensor requires approximately 3-5ms to complete a read operation.
     * Polling the IMU directly on the main thread would introduce significant latency.
     * To mitigate this, we utilize a dedicated thread for IMU data acquisition, allowing the main thread to continue without interruption.
     */
    public ThreadedIMU(RobotManager robotManager) {
        this.robotManager = robotManager;
        // Set the thread to be a daemon thread so that it will not prevent the program from exiting.
        setDaemon(true);
        setName("ThreadedIMU");
    }

    @Override
    public void run() {
        currentYaw = RobotManager.HardwareDevices.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        try {
            Thread.sleep(5);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
