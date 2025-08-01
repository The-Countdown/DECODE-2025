package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
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
    }

    static class ToggleCompetitionMode extends LinearOpMode {
        @Override
        public void runOpMode() {
            Status.competitionMode = !Status.competitionMode;

            while (opModeIsActive()) {
                telemetry.addData("Competition Mode is now", Status.competitionMode ? "ENABLED" : "DISABLED");
                telemetry.update();
            }
        }
    }
}