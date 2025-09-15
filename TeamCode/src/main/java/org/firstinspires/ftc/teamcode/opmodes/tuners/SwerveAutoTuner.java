package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Swerve Auto Tuner", group = "Tuner")
public class SwerveAutoTuner extends OpMode {
    public static double CURRENT_LOOP_TIME_AVG_MS;
    private RobotContainer robotContainer;
    public static double CURRENT_LOOP_TIME_MS;
    private final ElapsedTime targetTimer = new ElapsedTime();
    public boolean direction = true;
    double targetHeading = 0; // Set the target heading in degrees (e.g., 0 for a straight line)
    double previousError;
    long timer = System.currentTimeMillis();
    long timer2 = System.currentTimeMillis();
    long PIDTuneTime = 4000;
    private double averagedError;

    @Override
    public void init() {
        robotContainer = new RobotContainer(this);
        robotContainer.isRunning = true;
        robotContainer.init();
        robotContainer.indicatorLightFrontLeft.setColor(Constants.LED_COLOR.RED);
        robotContainer.refreshData();
        RobotContainer.HardwareDevices.imu.resetYaw();
        RobotContainer.HardwareDevices.pinpoint.resetPosAndIMU();
        robotContainer.opMode.telemetry.addLine("OpMode Initialized");
        robotContainer.opMode.telemetry.update();
        robotContainer.indicatorLightFrontLeft.setColor(Constants.LED_COLOR.GREEN);
    }

    @Override
    public void init_loop() {
        robotContainer.refreshData();
    }

    @Override
    public void start() {
        robotContainer.start(this);
        Status.opModeIsActive = true;
        targetTimer.reset();

        robotContainer.localizationUpdater.start();

        robotContainer.swerveModules[0].servo.setTargetAngle(90);
        robotContainer.swerveModules[1].servo.setTargetAngle(90);
        robotContainer.swerveModules[2].servo.setTargetAngle(90);
        robotContainer.swerveModules[3].servo.setTargetAngle(90);
    }

    @Override
    public void loop() {
        CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("teleOp");
        CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime("teleOp");
        robotContainer.refreshData();
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();

        if (System.currentTimeMillis() - timer >= 3000) {
            // Switch direction every 5 seconds
            if (direction) {
                robotContainer.swerveModules[0].motor.setTargetPower(0.4);
                robotContainer.swerveModules[1].motor.setTargetPower(0.4);
                robotContainer.swerveModules[2].motor.setTargetPower(0.4);
                robotContainer.swerveModules[3].motor.setTargetPower(0.4);
                direction = false;
            } else {
                robotContainer.swerveModules[0].motor.setTargetPower(-0.4);
                robotContainer.swerveModules[1].motor.setTargetPower(-0.4);
                robotContainer.swerveModules[2].motor.setTargetPower(-0.4);
                robotContainer.swerveModules[3].motor.setTargetPower(-0.4);
                direction = true;
            }
            timer = System.currentTimeMillis();
        }

        averagedError = Status.currentHeading - targetHeading + averagedError / 2;

        for (int i = 0; i < Constants.NUM_SWERVE_SERVOS; i++) {
            if (System.currentTimeMillis() - timer2 >= PIDTuneTime) {
                double kp = Constants.SWERVE_SERVO_KP[i];
                double ki = Constants.SWERVE_SERVO_KI[i];
                double kd = Constants.SWERVE_SERVO_KD[i];
                double kf = Constants.SWERVE_SERVO_KF[i];
                // Calculate the error and its derivative
                double error = averagedError;
                double derivative = (error - previousError) / 0.1;

                // Calculate a cost function (e.g., RMSE)
                double cost = Math.sqrt(Math.pow(error, 2) + Math.pow(derivative, 2));

                // Update the gains using gradient descent
                kp += 0.01 * (cost - kp);
                ki += 0.005 * (cost - ki);
                kd += 0.002 * (cost - kd);

                // Apply constraints to the gains
                if (kp < 0) kp = 0;
                if (ki < 0) ki = 0;
                if (kd < 0) kd = 0;

                previousError = error;

                if (error < -Math.PI) {
                    Constants.SWERVE_SERVO_KP[i] = kp;
                    Constants.SWERVE_SERVO_KI[i] = ki;
                    Constants.SWERVE_SERVO_KD[i] = kd;
                    Constants.SWERVE_SERVO_KF[i] = kf;
                } else if (error > Math.PI) {
                    Constants.SWERVE_SERVO_KP[i] = kp;
                    Constants.SWERVE_SERVO_KI[i] = ki;
                    Constants.SWERVE_SERVO_KD[i] = kd;
                    Constants.SWERVE_SERVO_KF[i] = kf;
                }

                if (i == 0) {
                    robotContainer.telemetry.addData("KP" + i, kp);
                    robotContainer.telemetry.addData("KI" + i, ki);
                    robotContainer.telemetry.addData("KD" + i, kd);
                    robotContainer.telemetry.addData("KF" + i, kf);
                }
                timer2 = System.currentTimeMillis();
            }
        }
        robotContainer.telemetry.addData("Timer", timer);
        robotContainer.telemetry.addData("Timer Diff", System.currentTimeMillis() - timer);
        robotContainer.telemetry.addData("Timer2", timer);
        robotContainer.telemetry.addData("Timer2 Diff", System.currentTimeMillis() - timer2);
        robotContainer.telemetry.addData("Current Heading", Status.currentHeading);
        robotContainer.telemetry.addData("Error", averagedError);


        robotContainer.telemetry.update();
    }
    
    @Override
    public void stop() {
        Status.opModeIsActive = false;
        robotContainer.isRunning = false;
    }
}

