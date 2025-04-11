package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.main.RobotManager;

public class Intake extends RobotManager.HardwareDevices {
    private final RobotManager robotManager;

    public Intake(RobotManager robotManager) {
        this.robotManager = robotManager;
    }

    public void setPower(double power) {
        RobotManager.HardwareDevices.intakeMotor.setPower(power);
    }

    public class ConveyorBelts {
        public void setLateralPower(double power) {
            RobotManager.HardwareDevices.lateralConveyorServo.setPower(power);
        }
        public void setLongitudinalPower(double power) {
            RobotManager.HardwareDevices.longitudinalConveyorServo.setPower(power);
        }
    }

}
