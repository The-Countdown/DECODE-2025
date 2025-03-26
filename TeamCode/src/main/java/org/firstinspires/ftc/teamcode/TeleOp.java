package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

public class TeleOp extends OpMode {

    public double voltage;
    public double current;

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        for (LynxModule hub : Robot.HardwareDevices.allHubs) {
            voltage = hub.getInputVoltage(VoltageUnit.VOLTS);
        }

        for (LynxModule hub : Robot.HardwareDevices.allHubs) {
            current = hub.getCurrent(CurrentUnit.MILLIAMPS);
        }
    }

    @Override
    public void stop() {

    }
}
