package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.hardware.*;

import java.util.ArrayList;

public class BetterThreadedHardwareManager {

    ArrayList<BetterThreadedDcMotor> motors = new ArrayList();
    ArrayList<BetterThreadedCRServo> crServos = new ArrayList();
    ArrayList<BetterThreadedServo> servos = new ArrayList();
    // ArrayList<BetterThreadedColorSensor> colorSensors = new ArrayList();
    // ArrayList<BetterThreadedTouchSensor> touchSensors = new ArrayList();
    // ArrayList<BetterThreadedAnalogInput> analogInputs = new ArrayList();

    public BetterThreadedHardwareManager() {
    }

    public void register(BetterThreadedDcMotor device) {
        this.motors.add(device);
    }

    public void register(BetterThreadedCRServo device) {
        this.crServos.add(device);
    }

    public void register(BetterThreadedServo device) {
        this.servos.add(device);
    }

    // public void register(BetterThreadedColorSensor device) {
    //     this.colorSensors.add(device);
    // }
    //
    // public void register(BetterThreadedTouchSensor device) {
    //     this.touchSensors.add(device);
    // }
    //
    // public void register(BetterThreadedAnalogInput device) {
    //     this.analogInputs.add(device);
    // }

    public void start() {
        for (BetterThreadedDcMotor motor : motors) {
            motor.enable();
            motor.start();
        }

        for (BetterThreadedCRServo crServo : crServos) {
            crServo.enable();
            crServo.start();
        }

        for (BetterThreadedServo servo : servos) {
            servo.enable();
            servo.start();
        }
    }

    public void stop() {
        for (BetterThreadedDcMotor motor : motors) {
            motor.disable();
            try {
                motor.join(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        for (BetterThreadedCRServo crServo : crServos) {
            crServo.disable();
            try {
                crServo.join(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        for (BetterThreadedServo servo : servos) {
            servo.disable();
            try {
                servo.join(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
