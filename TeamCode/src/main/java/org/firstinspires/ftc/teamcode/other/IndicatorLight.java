package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

import java.util.Objects;

public class IndicatorLight extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private final ServoImplEx indicatorLight;
    private final ElapsedTime flashingTimer = new ElapsedTime();
    private int count = 1;
    private double rainbowValue = 0;
    private int rainbowSign = 1;
    private final ElapsedTime rainbowTimer = new ElapsedTime();

    public IndicatorLight(RobotContainer robotContainer, ServoImplEx indicatorLight) {
        this.robotContainer = robotContainer;
        this.indicatorLight = indicatorLight;
    }

    public void setColor(Constants.LED_COLOR color) {
        indicatorLight.setPosition(Objects.requireNonNull(Constants.LED_COLOR_MAP.get(color)).ANALOG);
    }

    public void rainbow() {
        if (rainbowTimer.milliseconds() >= 2) {
            rainbowValue += (0.005 * rainbowSign);
            rainbowTimer.reset();
            if (rainbowValue >= 1) {
                rainbowSign = -1;
            } else if (rainbowValue <= 0) {
                rainbowSign = 1;
            }
        }

        indicatorLight.setPosition(scalePosition(rainbowValue));
    }

    public void rainbowReset() {
        rainbowValue = 0;
        rainbowSign = 1;
        rainbowTimer.reset();
    }

    public void flashing(Constants.LED_COLOR color, Constants.LED_COLOR colorTwo, double hz) {
        if (flashingTimer.seconds() <= 1/hz) {
            setColor(color);
        } else if (flashingTimer.seconds() <= 2/hz) {
            setColor(colorTwo);
        } else {
            flashingTimer.reset();
        }
    }

    public void flashing(Constants.LED_COLOR color, Constants.LED_COLOR colorTwo, double hz, int timesRepeated) {
        if (flashingTimer.seconds() <= 1/hz) {
            setColor(color);
        } else if (flashingTimer.seconds() <= 2/hz) {
            setColor(colorTwo);
        } else if (!(count >= timesRepeated)) {
            flashingTimer.reset();
            count += 1;
        }
    }

    public void flashingReset() {
        flashingTimer.reset();
        count = 1;
    }

    public void off() {
        setColor(Constants.LED_COLOR.OFF);
    }

    public double scalePosition(double position) {
        return (
                position *
                        (Objects.requireNonNull(Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.VIOLET)).ANALOG -
                                Objects.requireNonNull(Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.RED)).ANALOG)) +
                Objects.requireNonNull(Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.RED)).ANALOG;
    }
}
