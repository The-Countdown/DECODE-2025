package org.firstinspires.ftc.teamcode.other;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

import java.util.Objects;

public class IndicatorLight extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;

    public IndicatorLight(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setColor(Constants.LED_COLOR color) {
        RobotContainer.HardwareDevices.indicatorLight.setPosition(Objects.requireNonNull(Constants.LED_COLOR_MAP.get(color)).ANALOG);
    }

    public void off() {
        RobotContainer.HardwareDevices.indicatorLight.setPosition(Objects.requireNonNull(Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.OFF)).ANALOG);
    }

    public double scalePosition(double position) {
        return (
                position *
                        (Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.VIOLET).ANALOG -
                                Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.RED).ANALOG)) +
                Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.RED).ANALOG;
    }
}
