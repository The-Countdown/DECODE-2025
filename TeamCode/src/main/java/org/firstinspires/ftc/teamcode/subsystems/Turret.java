package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class Turret extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    public Turret(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void setIntakePower(double intakePower) {
        RobotContainer.HardwareDevices.turretIntakeServo.setPower(intakePower);
    }

    public void setTurretSpinPower(double power) {
        RobotContainer.HardwareDevices.turretRotation.setPower(power);
    }

    // the input is 0-1 not degrees, unless you convert it
<<<<<<< Updated upstream
    public void setArcAngle(double degrees) {
        RobotContainer.HardwareDevices.turretArcServo.setPosition(degrees); }

=======
    public void setArcAngle(double angle) { RobotManager.HardwareDevices.turretArcServo.setPosition(angle); }
>>>>>>> Stashed changes
    public class Flywheel {
        public void setPower(double power) {
            robotContainer.turretFlywheel.setPower(power);
        }
    }
}
