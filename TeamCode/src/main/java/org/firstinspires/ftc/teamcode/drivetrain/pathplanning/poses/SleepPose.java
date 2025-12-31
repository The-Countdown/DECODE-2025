package org.firstinspires.ftc.teamcode.drivetrain.pathplanning.poses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SleepPose extends GeneralPose {
    public double sleepTime; // In miliseconds
    public ElapsedTime sleepTimer = null;
    public SleepPose(double time) {
        this.sleepTime = time;
    }

    @Override
    public double getSleepTime() {
        return this.sleepTime;
    }
    @Override
    public boolean getDone() {
        if (this.sleepTimer == null) {
            this.sleepTimer = new ElapsedTime();
            this.sleepTimer.reset();
        }
        if (this.sleepTimer.milliseconds() > this.sleepTime) {
            this.sleepTimer = null;
            return true;
        }
        return false;
    }
}
