package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.main.RobotManager;

public class Turret extends RobotManager.HardwareDevices {
    private final RobotManager robotManager;
    public Turret(RobotManager robotManager) { this.robotManager = robotManager; }
    public class Flywheel {
        public void setPower(double power) { robotManager.turretFlywheel.setPower(power); }
    }
}
