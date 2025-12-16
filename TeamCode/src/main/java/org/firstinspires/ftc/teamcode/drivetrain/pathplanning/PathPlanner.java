package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

import java.util.ArrayList;
import java.lang.Thread;

// Coordinate graphs for reference. These all assume that you are looking at the zero heading orientation (Looking towards the goal assuming Teleop and Auto were started in the correct orientation).
//
// LimeLight
//      x
//      ^
// -y <- -> y
//      |
//     -x
//
// Teleop Pinpoint
//      y
//      ^
// x <-   -> -x
//      |
//     -y
//
//     There are a few more I need to write the rest.

public class PathPlanner {
    private Telemetry telemetry;
    ArrayList<GeneralPose> poses = new ArrayList<>();
    public int pointAmount = 0;
    private final RobotContainer robotContainer;
    public boolean pathCompleted = false;
    public int currentPath;
    public ArrayList<Double> estimatedPathTimes = new ArrayList<>();
    ArrayList<Double> powers = new ArrayList<>();
    ArrayList<Double> speeds = new ArrayList<>();
    ArrayList<Double> times = new ArrayList<>();
    double maxAccelerationDistance;

    public PathPlanner(Telemetry telemetry, RobotContainer robotContainer) {
        this.telemetry = telemetry;
        this.currentPath = 0;
        this.robotContainer = robotContainer;
    }

    public void addPose(Pose2D pose) {
        poses.add(new PositionPose(pose));
        pointAmount += 1;
    }

    public void addPose(double time) {
        poses.add(new SleepPose(time));
    }

    public boolean driveUsingPID(int index) {
        if (poses.get(index) instanceof PositionPose) {
            Status.targetPose = poses.get(index).getPose();
        } else if (poses.get(index) instanceof SleepPose) {
            return poses.get(index).getDone();
        }
        return PoseMath.isAtPos();
    }

    public void updatePathStatus(ElapsedTime pathTimer) {
        if (Status.currentPath == -1) {
            return;
        } else {
            Status.pathCompleted[Status.currentPath] = PoseMath.isAtPos();
            if (pathTimeOut(pathTimer)) {
                Status.pathCompleted[Status.currentPath] = true;
            }
        }
    }

    public void driveThroughPath () {
        if (!this.pathCompleted) {
            //if (driveUsingPID(this.currentPath) || pathTimeOut(poses.get(this.currentPath-1).getPose(), poses.get(this.currentPath).getPose(), pathTimer)) {
            if (driveUsingPID(this.currentPath)) {
                this.currentPath += 1;
                if (this.currentPath == this.poses.size()) {
                    this.pathCompleted = true;
                }
            }
        }
    }

    public void updatePathTimesAmount( ) {
        estimatedPathTimes.clear();
        Status.pathsToCalculate = pointAmount;
    }

    public void updatePathTimes() {
        for (int i = Status.pathsToCalculate; i > 0; i--) {
            estimatedPathTimes.add(calculateEstimatedPathTime(i));
            Status.pathsToCalculate--;        }
    }

    public boolean pathTimeOut(ElapsedTime pathTimer){
        return pathTimer.milliseconds() > estimatedPathTimes.get(currentPath) + Constants.Pathing.PATH_TIMEOUT_ERROR_MS;
    }

    public double getCurrentPathTime() {
        return estimatedPathTimes.get(currentPath) + Constants.Pathing.PATH_TIMEOUT_ERROR_MS;
    }

    public double calculateEstimatedPathTime(int path) {
        maxAccelerationDistance = Constants.Pathing.ACCELERATION_TABLE.lastKey();
        if (Status.pathsToCalculate < 1) return 0;
        Status.splitsToCalculate = Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;
        Pose2D startingPathPose;
        if (poses.get(path) instanceof SleepPose) {
             startingPathPose = poses.get(path-1).getPose();
        } else {
         startingPathPose = poses.get(path).getPose();
        }
        if (poses.get(path+1) instanceof SleepPose) {
            return (poses.get(path+1).getSleepTime());
        }
        Pose2D endingPathPose = poses.get(path+1).getPose();

        double xDiff = endingPathPose.getX(DistanceUnit.CM) - startingPathPose.getX(DistanceUnit.CM);
        double yDiff = endingPathPose.getY(DistanceUnit.CM) - startingPathPose.getY(DistanceUnit.CM);
        double hDiff = HelperFunctions.normalizeAngle(endingPathPose.getHeading(AngleUnit.DEGREES) - startingPathPose.getHeading(AngleUnit.DEGREES));
        double totalDiff = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));

        double xSplit = xDiff / Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;
        double ySplit = yDiff / Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;
        double hSplit = hDiff / Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;
        double splitDist = totalDiff / Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;

        double remainingAccelerationDistance = maxAccelerationDistance;

        double lastXError = 0;
        double lastYError = 0;
        double lastHError = 0;
        double currentTime = 0;
        int iOffset = 0;
        int splitNum = Constants.Pathing.PATH_NUM_OF_SPLITS_FOR_ESTIMATED_TIME;

        while (Status.splitsToCalculate > 0 && Status.opModeIsActive) {
            powers.add(robotContainer.drivetrain.fakePowerInput(
                        HelperFunctions.clamp(robotContainer.latitudePID.fakeCalculate(xDiff, currentTime, lastXError), -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER),
                        HelperFunctions.clamp(robotContainer.longitudePID.fakeCalculate(yDiff, currentTime, lastYError), -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER),
                        HelperFunctions.clamp(robotContainer.headingPID.fakeCalculate(hDiff, currentTime, lastHError), -Constants.Pathing.SWERVE_MAX_POWER, Constants.Pathing.SWERVE_MAX_POWER)
            )[1]);

            if (remainingAccelerationDistance > maxAccelerationDistance - (maxAccelerationDistance * powers.get(Math.abs(Status.splitsToCalculate-splitNum)))) {
                currentTime += accelerationTableInterpolation(Math.abs(Status.splitsToCalculate-splitNum)*splitDist)-accelerationTableInterpolation(Math.abs(Status.splitsToCalculate-splitNum-1)*splitDist);
                iOffset++;
            } else {
                //cm per second
                speeds.add((ticksPerSecondOfMotor(powers.get(Math.abs(Status.splitsToCalculate-splitNum)-iOffset)) / Constants.Swerve.MOTOR_TICKS_PER_REVOLUTION) * Constants.Swerve.MOTOR_TO_WHEEL_GEAR_RATIO * 2 * Math.PI * (Constants.Robot.WHEEL_DIAMETER_MM / 10));
                times.add(splitDist/speeds.get(Math.abs(Status.splitsToCalculate-splitNum)-iOffset));
                currentTime += times.get(Math.abs(Status.splitsToCalculate-splitNum)-iOffset);
            }

            lastXError = xDiff;
            lastYError = yDiff;
            lastHError = hDiff;

            xDiff += Math.signum(xDiff) == 1 ? -xSplit : xSplit;
            yDiff += Math.signum(yDiff) == 1 ? -ySplit : ySplit;
            hDiff += Math.signum(hDiff) == 1 ? -hSplit : hSplit;

            remainingAccelerationDistance -= splitDist;

            if (Math.abs(hDiff) < 0.1 && Math.abs(yDiff) < 0.1 && Math.abs(xDiff) < 0.1) break;
            Status.splitsToCalculate--;
        }
        //The total time to complete the path = currentTime;
        return currentTime;
    }

    public double ticksPerSecondOfMotor(double power){
        return Constants.Pathing.SWERVE_MAX_VELOCITY * power / 0.8;
    }

    public double accelerationTableInterpolation(double distance) {
        distance = Math.abs(distance);
        double distanceLower;
        double distanceUpper;

        if (Constants.Pathing.ACCELERATION_TABLE.floorKey(distance) == null) {
            distanceLower = Constants.Pathing.ACCELERATION_TABLE.ceilingKey(distance);
            distanceUpper = Constants.Pathing.ACCELERATION_TABLE.higherKey(distanceLower);
        } else if (Constants.Pathing.ACCELERATION_TABLE.ceilingKey(distance) == null) {
            distanceUpper = Constants.Pathing.ACCELERATION_TABLE.floorKey(distance);
            distanceLower = Constants.Pathing.ACCELERATION_TABLE.lowerKey(distanceUpper);
        } else {
            distanceLower = Constants.Pathing.ACCELERATION_TABLE.floorKey(distance);
            distanceUpper = Constants.Pathing.ACCELERATION_TABLE.ceilingKey(distance);
        }

        return HelperFunctions.interpolate(
                Constants.Pathing.ACCELERATION_TABLE.get(distanceLower),
                Constants.Pathing.ACCELERATION_TABLE.get(distanceUpper),
                (distance - distanceLower) / (distanceUpper - distanceLower)
                );
    }

    public void accelTableInit() {
        Constants.Pathing.ACCELERATION_TABLE.put(0.0, 0.001119417);
        Constants.Pathing.ACCELERATION_TABLE.put(1.22E-4, 0.010142418);
        Constants.Pathing.ACCELERATION_TABLE.put(1.27E-4, 0.256907901);
        Constants.Pathing.ACCELERATION_TABLE.put(0.02387704717, 0.289432529);
        Constants.Pathing.ACCELERATION_TABLE.put(0.0844440415, 0.302068114);
        Constants.Pathing.ACCELERATION_TABLE.put(0.2080164947, 0.316093782);
        Constants.Pathing.ACCELERATION_TABLE.put(0.2785114319, 0.330841325);
        Constants.Pathing.ACCELERATION_TABLE.put(0.4150996399, 0.341729826);
        Constants.Pathing.ACCELERATION_TABLE.put(0.5273043356, 0.355914453);
        Constants.Pathing.ACCELERATION_TABLE.put(0.7699388166, 0.370747746);
        Constants.Pathing.ACCELERATION_TABLE.put(1.096985101, 0.383611122);
        Constants.Pathing.ACCELERATION_TABLE.put(1.440582667, 0.404621916);
        Constants.Pathing.ACCELERATION_TABLE.put(1.99949841, 0.41499825);
        Constants.Pathing.ACCELERATION_TABLE.put(2.555890117, 0.426804043);
        Constants.Pathing.ACCELERATION_TABLE.put(3.081549089, 0.441006169);
        Constants.Pathing.ACCELERATION_TABLE.put(3.732233072, 0.452901212);
        Constants.Pathing.ACCELERATION_TABLE.put(4.328778494, 0.463218338);
        Constants.Pathing.ACCELERATION_TABLE.put(5.076926861, 0.480776965);
        Constants.Pathing.ACCELERATION_TABLE.put(6.187361818, 0.496701967);
        Constants.Pathing.ACCELERATION_TABLE.put(7.285340207, 0.512125593);
        Constants.Pathing.ACCELERATION_TABLE.put(8.141033077, 0.523320344);
        Constants.Pathing.ACCELERATION_TABLE.put(8.993919552, 0.535435887);
        Constants.Pathing.ACCELERATION_TABLE.put(10.26841768, 0.549358014);
        Constants.Pathing.ACCELERATION_TABLE.put(11.17095154, 0.568576516);
        Constants.Pathing.ACCELERATION_TABLE.put(12.48396512, 0.573125058);
        Constants.Pathing.ACCELERATION_TABLE.put(13.46030259, 0.592990185);
        Constants.Pathing.ACCELERATION_TABLE.put(15.23856768, 0.607546103);
        Constants.Pathing.ACCELERATION_TABLE.put(16.63236694, 0.621510229);
        Constants.Pathing.ACCELERATION_TABLE.put(20.20446786, 0.654217149);
        Constants.Pathing.ACCELERATION_TABLE.put(22.1621444, 0.674590943);
        Constants.Pathing.ACCELERATION_TABLE.put(23.82482743, 0.688486236);
        Constants.Pathing.ACCELERATION_TABLE.put(25.27798363, 0.701437404);
        Constants.Pathing.ACCELERATION_TABLE.put(26.70006374, 0.718370114);
        Constants.Pathing.ACCELERATION_TABLE.put(28.21921039, 0.730558865);
        Constants.Pathing.ACCELERATION_TABLE.put(30.35498225, 0.745548492);
        Constants.Pathing.ACCELERATION_TABLE.put(31.86522721, 0.755847826);
        Constants.Pathing.ACCELERATION_TABLE.put(33.40366797, 0.782164912);
        Constants.Pathing.ACCELERATION_TABLE.put(35.39930415, 0.791506996);
        Constants.Pathing.ACCELERATION_TABLE.put(37.61517333, 0.807065373);
        Constants.Pathing.ACCELERATION_TABLE.put(39.34212084, 0.829578542);
        Constants.Pathing.ACCELERATION_TABLE.put(41.63619712, 0.840949751);
        Constants.Pathing.ACCELERATION_TABLE.put(43.71109329, 0.849803294);
        Constants.Pathing.ACCELERATION_TABLE.put(44.94755544, 0.867302712);
        Constants.Pathing.ACCELERATION_TABLE.put(46.97862387, 0.877710838);
        Constants.Pathing.ACCELERATION_TABLE.put(48.38007119, 0.891334298);
        Constants.Pathing.ACCELERATION_TABLE.put(50.15843643, 0.906909883);
        Constants.Pathing.ACCELERATION_TABLE.put(52.16376289, 0.922277218);
        Constants.Pathing.ACCELERATION_TABLE.put(54.17024877, 0.934941094);
        Constants.Pathing.ACCELERATION_TABLE.put(55.89481438, 0.950341971);
        Constants.Pathing.ACCELERATION_TABLE.put(57.85517037, 0.963317347);
        Constants.Pathing.ACCELERATION_TABLE.put(59.42051888, 0.97461739);
        Constants.Pathing.ACCELERATION_TABLE.put(61.42285705, 0.994403183);
        Constants.Pathing.ACCELERATION_TABLE.put(63.54781536, 1.013996769);
        Constants.Pathing.ACCELERATION_TABLE.put(65.65676567, 1.021599936);
        Constants.Pathing.ACCELERATION_TABLE.put(67.8459511, 1.04058248);
        Constants.Pathing.ACCELERATION_TABLE.put(69.83109745, 1.053862356);
        Constants.Pathing.ACCELERATION_TABLE.put(72.14480638, 1.068342149);
        Constants.Pathing.ACCELERATION_TABLE.put(73.58416128, 1.081564275);
        Constants.Pathing.ACCELERATION_TABLE.put(75.39251063, 1.090707151);
        Constants.Pathing.ACCELERATION_TABLE.put(76.69092498, 1.111778028);
        Constants.Pathing.ACCELERATION_TABLE.put(78.88724105, 1.116347862);
        Constants.Pathing.ACCELERATION_TABLE.put(80.49663221, 1.133063864);
        Constants.Pathing.ACCELERATION_TABLE.put(82.81404458, 1.147285824);
        Constants.Pathing.ACCELERATION_TABLE.put(85.04234662, 1.167323617);
        Constants.Pathing.ACCELERATION_TABLE.put(87.15059188, 1.179070785);
        Constants.Pathing.ACCELERATION_TABLE.put(89.15502968, 1.193455203);
        Constants.Pathing.ACCELERATION_TABLE.put(91.17662683, 1.209172247);
        Constants.Pathing.ACCELERATION_TABLE.put(93.15733497, 1.223684706);
        Constants.Pathing.ACCELERATION_TABLE.put(95.24081723, 1.237119749);
        Constants.Pathing.ACCELERATION_TABLE.put(96.94475969, 1.252303043);
        Constants.Pathing.ACCELERATION_TABLE.put(98.55988757, 1.261650377);
        Constants.Pathing.ACCELERATION_TABLE.put(100.5334595, 1.274536212);
        Constants.Pathing.ACCELERATION_TABLE.put(102.1004999, 1.284728796);
        Constants.Pathing.ACCELERATION_TABLE.put(103.948067, 1.301930131);
        Constants.Pathing.ACCELERATION_TABLE.put(105.8149488, 1.316301716);
        Constants.Pathing.ACCELERATION_TABLE.put(107.7659278, 1.325648175);
        Constants.Pathing.ACCELERATION_TABLE.put(109.5482043, 1.34160176);
        Constants.Pathing.ACCELERATION_TABLE.put(111.2886082, 1.354291595);
        Constants.Pathing.ACCELERATION_TABLE.put(113.062733, 1.369604096);
        Constants.Pathing.ACCELERATION_TABLE.put(114.6318929, 1.379925597);
        Constants.Pathing.ACCELERATION_TABLE.put(116.6176394, 1.394413849);
        Constants.Pathing.ACCELERATION_TABLE.put(118.5789105, 1.407025808);
        Constants.Pathing.ACCELERATION_TABLE.put(120.7288266, 1.42903556);
        Constants.Pathing.ACCELERATION_TABLE.put(122.5963226, 1.437101895);
        Constants.Pathing.ACCELERATION_TABLE.put(124.6320039, 1.450800021);
        Constants.Pathing.ACCELERATION_TABLE.put(126.4185811, 1.469840898);
        Constants.Pathing.ACCELERATION_TABLE.put(128.7374866, 1.485793899);
        Constants.Pathing.ACCELERATION_TABLE.put(130.8923659, 1.497098026);
        Constants.Pathing.ACCELERATION_TABLE.put(132.6222743, 1.513445652);
        Constants.Pathing.ACCELERATION_TABLE.put(134.8310892, 1.525264862);
        Constants.Pathing.ACCELERATION_TABLE.put(136.9713889, 1.541865072);
        Constants.Pathing.ACCELERATION_TABLE.put(138.7013669, 1.55403924);
        Constants.Pathing.ACCELERATION_TABLE.put(140.4547351, 1.563229657);
        Constants.Pathing.ACCELERATION_TABLE.put(142.1246875, 1.577186492);
        Constants.Pathing.ACCELERATION_TABLE.put(143.7875081, 1.588724243);
        Constants.Pathing.ACCELERATION_TABLE.put(145.4450362, 1.603298245);
        Constants.Pathing.ACCELERATION_TABLE.put(147.5674636, 1.614530912);
        Constants.Pathing.ACCELERATION_TABLE.put(149.4539823, 1.629337955);
        Constants.Pathing.ACCELERATION_TABLE.put(151.3444447, 1.647650541);
        Constants.Pathing.ACCELERATION_TABLE.put(153.8515268, 1.659675084);
        Constants.Pathing.ACCELERATION_TABLE.put(156.0453913, 1.679376586);
        Constants.Pathing.ACCELERATION_TABLE.put(157.7272967, 1.688017795);
        Constants.Pathing.ACCELERATION_TABLE.put(159.8656019, 1.707087547);
        Constants.Pathing.ACCELERATION_TABLE.put(161.8972524, 1.719041506);
        Constants.Pathing.ACCELERATION_TABLE.put(163.9156222, 1.734041924);
        Constants.Pathing.ACCELERATION_TABLE.put(165.2388064, 1.751384718);
        Constants.Pathing.ACCELERATION_TABLE.put(167.3004275, 1.765063011);
        Constants.Pathing.ACCELERATION_TABLE.put(169.0743778, 1.780410804);
        Constants.Pathing.ACCELERATION_TABLE.put(170.8597274, 1.79030968);
        Constants.Pathing.ACCELERATION_TABLE.put(172.0673581, 1.813683849);
        Constants.Pathing.ACCELERATION_TABLE.put(173.9384587, 1.823523225);
        Constants.Pathing.ACCELERATION_TABLE.put(176.0765487, 1.842899519);
        Constants.Pathing.ACCELERATION_TABLE.put(177.7886224, 1.857168145);
        Constants.Pathing.ACCELERATION_TABLE.put(179.3433807, 1.870718396);
    }

    public void waitForTarget() {
        while (!PoseMath.isAtPos()){
            Thread.yield();
        }
    }

    public void displayPoses() {
        for (int i = 0; i < poses.size(); i++) {
            telemetry.addData("Pose", poses.get(i).toString());
            telemetry.update();
        }
    }

    public void clearPoses() {
        this.poses.clear();
        pointAmount = 0;
    }

    public class GeneralPose {
        public GeneralPose() {
        }

        public Pose2D getPose() {
            return null;
        }

        public boolean getDone() {
            return false;
        }

        public double getSleepTime() {
            return 0;
        }
    }

    public class PositionPose extends GeneralPose {
        private Pose2D pose;

        public PositionPose(Pose2D pose) {
            this.pose = pose;
        }

        @Override
        public Pose2D getPose() {
            return this.pose;
        }
    }

    public class SleepPose extends GeneralPose {
        public double sleepTime; // In miliseconds
        public ElapsedTime sleepTimer = null;
        public SleepPose(double time) {
            this.sleepTime = time;
        }

        @Override
        public double getSleepTime() {
            return this.sleepTime;
        }
        @Override
        public boolean getDone() {
            if (this.sleepTimer == null) {
                this.sleepTimer = new ElapsedTime();
                this.sleepTimer.reset();
            }
            if (this.sleepTimer.milliseconds() > this.sleepTime) {
                this.sleepTimer = null;
                return true;
            }
            return false;
        }
    }
}
