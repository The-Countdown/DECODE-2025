package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class BetterAnalogInput {
    
    private static AnalogInput analog;

    private long lastTime;

    private double lastVoltage = 0;
    private double voltage = 0;

    public BetterAnalogInput(AnalogInput analog) {
        this.analog = analog;
    }

    public void updateGetVoltage(double voltage, int minTimeBetweenUpdates) {
        this.voltage = voltage;
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTime >= minTimeBetweenUpdates && voltage != lastVoltage) {
            voltage = analog.getVoltage();
            lastTime = System.currentTimeMillis();
            this.lastVoltage = voltage;
        }
    }

    public double getVoltage() {
        return voltage;
    }
    
    public String getConnectionInfo() {
        return analog.getConnectionInfo();
    }
}
