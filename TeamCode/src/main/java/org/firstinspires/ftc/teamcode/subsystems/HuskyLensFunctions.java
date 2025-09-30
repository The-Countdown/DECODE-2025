package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

import java.lang.reflect.Array;

public class HuskyLensFunctions {
    private RobotContainer robotContainer;

    private HuskyLens huskyLens;


    public enum Color {
        GREEN,
        PURPLE,
        NONE;
    }
    public HuskyLensFunctions(RobotContainer robotContainer, HuskyLens lens) {
        this.robotContainer = robotContainer;
        this.huskyLens = lens;
    }

    public Color checkColor() {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if(blocks.length == 0){
            robotContainer.telemetry.addData("Seen ball: ", Color.NONE);
            return Color.NONE;
        }else {
            if (blocks[0].id == 2) {
                robotContainer.telemetry.addData("Seen ball: ", Color.PURPLE);
                return Color.PURPLE;
            } else if (blocks[0].id == 1) {
                robotContainer.telemetry.addData("Seen ball: ", Color.GREEN);
                return Color.GREEN;
            } else {
                robotContainer.telemetry.addData("Seen ball: ", Color.NONE);
                return Color.NONE;
            }
        }
    }
}