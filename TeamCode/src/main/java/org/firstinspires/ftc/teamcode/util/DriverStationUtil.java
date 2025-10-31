package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.Status;

public class DriverStationUtil {

    private static OpModeMeta metaForClass(String name) {
        return new OpModeMeta.Builder()
                .setName(name)
                .setGroup("Utilities")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        manager.register(metaForClass("Toggle Competition Mode"), new ToggleCompetitionMode());
        manager.register(metaForClass("Alliance Switch"), new AllianceSwitch());
    }

    static class ToggleCompetitionMode extends LinearOpMode {
        @Override
        public void runOpMode() {
            Status.competitionMode = !Status.competitionMode;

            waitForStart();

            while (opModeIsActive()) {
                telemetry.addData("Competition Mode is now", Status.competitionMode ? "ENABLED" : "DISABLED");
                telemetry.update();
            }
        }
    }

    static class AllianceSwitch extends LinearOpMode {
        @Override
        public void runOpMode() {
            Status.alliance = Status.alliance == Constants.Game.ALLIANCE.BLUE ? Constants.Game.ALLIANCE.RED : Constants.Game.ALLIANCE.BLUE;

            waitForStart();

            while (opModeIsActive()) {
                telemetry.addData("Alliance is now", Status.alliance == Constants.Game.ALLIANCE.BLUE ? "BLUE" : "RED");
                telemetry.update();
            }
        }
    }
}