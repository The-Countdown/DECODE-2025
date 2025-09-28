package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class ADGUpdater extends Thread {
    public ADG728 mux;
    private RobotContainer robotContainer;

    public ADGUpdater(ADG728 mux, RobotContainer robotContainer) {
        this.mux = mux;
        // Set the thread to be a daemon thread so that it will not prevent the program from exiting.
        setDaemon(true);
        setName("ADGUpdater");
    }

    @Override
    public void run() {
        while(true) {
            for (int i = 1; i <= 2; i++) {
                mux.setPort(i);
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mux.saveVoltage();
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
}