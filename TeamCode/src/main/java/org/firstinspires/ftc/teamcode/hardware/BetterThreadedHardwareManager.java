package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.hardware.*;

import java.util.ArrayList;

public class BetterThreadedHardwareManager {

    ArrayList<BetterThreadedDcMotor> motors = new ArrayList();
    ArrayList<BetterThreadedCRServo> crServos = new ArrayList();
    ArrayList<BetterThreadedServo> servos = new ArrayList();
    ArrayList<BetterThreadedColorSensor> colorSensors = new ArrayList();
    ArrayList<BetterThreadedTouchSensor> touchSensors = new ArrayList();
    ArrayList<BetterThreadedAnalogInput> analogInputs = new ArrayList();

    BetterThreadedHardwareManager {
    }

    public void register(BetterThreadDcMotor device) {
        this.motors.add(device);
    }

    public void register(BetterThreadServo device) {
        this.servos.add(device);
    }

    public void register(BetterThreadCRServo device) {
        this.crServos.add(device);
    }

    public void register(BetterThreadColorSensor device) {
        this.colorSensors.add(device);
    }

    public void register(BetterThreadTouchSensor device) {
        this.touchSensors.add(device);
    }

    public void register(BetterThreadAnalogInput device) {
        this.analogInputs.add(device);
    }

}
