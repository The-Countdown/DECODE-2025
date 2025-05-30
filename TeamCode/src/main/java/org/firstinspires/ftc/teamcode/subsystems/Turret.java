package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class Turret extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;

    public Turret(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setIntakePower(double intakePower) {
        robotContainer.testCriticalHardwareDevice(RobotContainer.HardwareDevices.turretIntakeServo);
        RobotContainer.HardwareDevices.turretIntakeServo.setPower(intakePower);
    }

    public void setTurretSpinPower(double power) {
        robotContainer.testCriticalHardwareDevice(RobotContainer.HardwareDevices.turretRotation);
        RobotContainer.HardwareDevices.turretRotation.setPower(power);
    }

    public void setArcAngle(double angle) {
        robotContainer.testCriticalHardwareDevice(RobotContainer.HardwareDevices.turretArcServo);
        RobotContainer.HardwareDevices.turretArcServo.setPosition(angle);
    }

    public class Flywheel {
        public void setPower(double power) {
            robotContainer.turretFlywheel.setPower(power);
        }

        public void setVelocity(double velocity) {
            robotContainer.turretFlywheel.setVelocity(velocity);
        }
    }

    public final Flywheel flywheel = new Flywheel();
}
