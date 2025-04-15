package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class Intake extends RobotContainer.HardwareDevices {
    private final RobotContainer robotManager;

    public Intake(RobotContainer robotManager) {
        this.robotManager = robotManager;
    }

    public void setPower(double power) {
        RobotContainer.HardwareDevices.intakeMotor.setPower(power);
    }

    public class ConveyorBelts {
        public void setLateralPower(double power) {
            RobotContainer.HardwareDevices.lateralConveyorServo.setPower(power);
        }
        public void setLongitudinalPower(double power) {
            RobotContainer.HardwareDevices.longitudinalConveyorServo.setPower(power);
        }
    }

}
