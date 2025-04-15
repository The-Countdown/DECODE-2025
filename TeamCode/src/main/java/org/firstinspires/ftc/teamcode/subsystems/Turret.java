package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class Turret extends RobotContainer.HardwareDevices {
    private final RobotContainer robotManager;
    public Turret(RobotContainer robotManager) {
        this.robotManager = robotManager;
    }
    //        Turret:
    //            - rotateTo(degrees) // Clockwise should be positive, use the absolute encoder

    public void setIntakePower(double intakePower) {
        RobotContainer.HardwareDevices.turretIntakeServo.setPower(intakePower);
    }

    // the input is 0-1 not degrees, unless you convert it
    public void setArcAngle(double degrees) {
        RobotContainer.HardwareDevices.turretArcServo.setPosition(degrees); }

    public class Flywheel {
        public void setPower(double power) {
            robotManager.turretFlywheel.setPower(power);
        }
    }
}
