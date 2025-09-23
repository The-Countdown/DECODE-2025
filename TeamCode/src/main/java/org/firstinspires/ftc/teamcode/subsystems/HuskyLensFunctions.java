package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HuskyLensFunctions {
    private HuskyLens huskyLens;
    enum Color {
        GREEN,
        PURPLE,
        NONE;
    }
    public HuskyLensFunctions(HardwareMap hardwareMap) {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
    }

    public Color checkColor(Color color) {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if(blocks.length > 0){
            if (blocks[0].id == 1) {
                return Color.GREEN;
            }else {
                return Color.PURPLE;
            }
        } else{
            return Color.NONE;
        }
    }
}