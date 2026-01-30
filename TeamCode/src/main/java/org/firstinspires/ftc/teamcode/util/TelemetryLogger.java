package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

public class TelemetryLogger extends Thread {
    private final RobotContainer robotContainer;
    public double CURRENT_LOOP_TIME_MS;
    public double CURRENT_LOOP_TIME_AVG_MS;
    public boolean enabled;
    private ElapsedTime timer = new ElapsedTime();

    /**
     * The pinpoint takes more time than a normal device, however, I don't know how much that is,
     * so I put it into a thread to be safe.
     */
    public TelemetryLogger(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.CURRENT_LOOP_TIME_AVG_MS = 0;
        this.CURRENT_LOOP_TIME_MS = 0;
        this.enabled = true;
        // Set the thread to be a daemon thread so that it will not prevent the program from exiting.
        setDaemon(true);
        setName("telemetryLogger");
    }

    @Override
    public void run() {
        if (!enabled) {
            return;
        }
        while (Status.opModeIsActive) {
            CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("telemetryLogger");
            CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime("telemetryLogger");
            // if (robotContainer.gamepadEx2.dpadDown.wasJustPressed() && timer.milliseconds() > 2000) {
            //     robotContainer.writeDataLog();
            //     robotContainer.writeEventLog();
            //     timer.reset();
            // }
            Thread.yield();
        }
        Thread.yield();
    }

    public void stopThread() {
        enabled = false;
    }
}
