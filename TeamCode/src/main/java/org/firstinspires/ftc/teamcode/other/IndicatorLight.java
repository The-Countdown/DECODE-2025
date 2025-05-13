package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

import java.util.Objects;

public class IndicatorLight extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private double rainbowValue = 0;
    private final ElapsedTime rainbowTimer = new ElapsedTime();

    public IndicatorLight(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setColor(Constants.LED_COLOR color) {
        RobotContainer.HardwareDevices.indicatorLight.setPosition(Objects.requireNonNull(Constants.LED_COLOR_MAP.get(color)).ANALOG);
    }

    public void rainbow() {
        if (rainbowTimer.milliseconds() >= 2) {
            rainbowValue += 0.001;
            rainbowTimer.reset();
            if (rainbowValue >= 1) {
                rainbowValue = 0;
            }
        }

        RobotContainer.HardwareDevices.indicatorLight.setPosition(scalePosition(rainbowValue));
    }

    public void off() {
        RobotContainer.HardwareDevices.indicatorLight.setPosition(Objects.requireNonNull(Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.OFF)).ANALOG);
    }

    public double scalePosition(double position) {
        return (
                position *
                        (Objects.requireNonNull(Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.VIOLET)).ANALOG -
                                Objects.requireNonNull(Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.RED)).ANALOG)) +
                Objects.requireNonNull(Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.RED)).ANALOG;
    }
}
