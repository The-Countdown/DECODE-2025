package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class ColorSensorFunctions {
    private RobotContainer robotContainer;
    private RevColorSensorV3 colorSensor;

    public ColorSensorFunctions(RobotContainer robotContainer, RevColorSensorV3 colorSensor) {
        this.robotContainer = robotContainer;
        this.colorSensor = colorSensor;
    }


}
