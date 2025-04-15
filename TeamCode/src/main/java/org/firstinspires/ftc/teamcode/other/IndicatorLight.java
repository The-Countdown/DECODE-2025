package org.firstinspires.ftc.teamcode.other;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

import java.util.Objects;

public class IndicatorLight extends RobotContainer.HardwareDevices {
    private final RobotContainer robotManager;

    public IndicatorLight(RobotContainer robotManager) {
        this.robotManager = robotManager;
    }

    public void setColor(Constants.LED_COLOR color) {
        RobotContainer.HardwareDevices.indicatorLight.setPosition(Objects.requireNonNull(Constants.LED_COLOR_MAP.get(color)).ANALOG);
    }

    public void off() {
        RobotContainer.HardwareDevices.indicatorLight.setPosition(Objects.requireNonNull(Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.OFF)).ANALOG);
    }


}
