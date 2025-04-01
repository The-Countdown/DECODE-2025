package org.firstinspires.ftc.teamcode;

public class IndicatorLight extends RobotManager.HardwareDevices {
    private final RobotManager robotManager;

    public IndicatorLight(RobotManager robotManager) {
        this.robotManager = robotManager;
    }

    public void setColor(Constants.LED_COLOR color) {
        RobotManager.HardwareDevices.indicatorLight.setPosition(Constants.LED_COLOR_MAP.get(color).ANALOG);
    }

    public void off() {
        RobotManager.HardwareDevices.indicatorLight.setPosition(Constants.LED_COLOR_MAP.get(Constants.LED_COLOR.OFF).ANALOG);
    }


}
