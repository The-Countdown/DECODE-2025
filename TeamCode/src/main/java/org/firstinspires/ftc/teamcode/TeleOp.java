package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

public class TeleOp extends OpMode {
    private Robot robot = new Robot(this);

    @Override
    public void init() {
        robot.running = true;
    }

    @Override
    public void init_loop() {
        robot.refreshData();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robot.refreshData();
    }

    @Override
    public void stop() {
        robot.running = false;
    }
}
