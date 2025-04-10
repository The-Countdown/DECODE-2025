package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.main.RobotManager;

public class Turret extends RobotManager.HardwareDevices {
    private final RobotManager robotManager;
    public Turret(RobotManager robotManager) { this.robotManager = robotManager; }
    //        Turret:
    //            - rotateTo(degrees) // Clockwise should be positive, use the absolute encoder

    public void setIntakePower(double intakePower) { RobotManager.HardwareDevices.turretIntakeServo.setPower(intakePower);  }

    // the input is 0-1 not degrees, unless you convert it
    public void setArcAngle(double degrees) { RobotManager.HardwareDevices.turretArcServo.setPosition(degrees); }
    public class Flywheel {
        public void setPower(double power) { robotManager.turretFlywheel.setPower(power); }
    }
}
