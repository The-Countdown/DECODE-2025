package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class BetterAnalogInput {
    private AnalogInput analog;

    private long lastTime;

    private double voltage = 0;

    private double minTimeBetweenUpdates = 0;

    public BetterAnalogInput(AnalogInput analog, int minTimeBetweenUpdates) {
        this.analog = analog;
        this.minTimeBetweenUpdates = HelperFunctions.getRandomWithin(minTimeBetweenUpdates, 0.5);
    }

    public double updateGetVoltage() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTime >= minTimeBetweenUpdates) {
            this.voltage = analog.getVoltage();
            lastTime = System.currentTimeMillis();
        }
        return this.voltage;
    }

    public String getConnectionInfo() {
        return analog.getConnectionInfo();
    }
}
