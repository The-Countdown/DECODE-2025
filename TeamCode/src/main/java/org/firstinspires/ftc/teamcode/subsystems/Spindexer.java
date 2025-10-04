package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class Spindexer {
    private final RobotContainer robotContainer;
    private final CRServoImplEx spindexerServo;

    public Spindexer (RobotContainer robotContainer, CRServoImplEx spindexerServo) {
        this.robotContainer = robotContainer;
        this.spindexerServo = spindexerServo;
    }

    public void setPower(double power) {
        spindexerServo.setPower(power);
    }
}
