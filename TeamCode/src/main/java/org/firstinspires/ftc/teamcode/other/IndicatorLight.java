package org.firstinspires.ftc.teamcode.other;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotManager;

import java.util.Objects;

public class IndicatorLight extends RobotManager.HardwareDevices {
    private final RobotManager robotManager;

    public IndicatorLight(RobotManager robotManager) {
        this.robotManager = robotManager;
    }

    public void setColor(Constants.LED_COLOR color) {
        RobotManager.HardwareDevices.indicatorLight.setPosition(Objects.requireNonNull(Constants.LED_COLOR_MAP.get(color)).ANALOG);
    }

    public void off() {
        RobotManager.HardwareDevices.indicatorLight.setPosition(Objects.requireNonNull(Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.OFF)).ANALOG);
    }


}
