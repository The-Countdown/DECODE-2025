package org.firstinspires.ftc.teamcode.drivetrain.pathplanning;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.NavigableMap;

public class FeedForward {

    public class TrajectoryPoint {
        public Pose2D pose;
        public double velocity; // cm/s
        public double time; // seconds

        public TrajectoryPoint(Pose2D pose, double velocity, double time) {
            this.pose = pose;
            this.velocity = velocity;
            this.time = time;
        }
    }

    public List<TrajectoryPoint> generatePath(Pose2D startPose, Pose2D endPose, double dt) {

        List<TrajectoryPoint> path = new ArrayList<>();
        NavigableMap<Double, Double> accelMap = Constants.Pathing.ACCELERATION_TABLE;

        double deltaX = endPose.getX(DistanceUnit.CM) - startPose.getX(DistanceUnit.CM);
        double deltaY = endPose.getY(DistanceUnit.CM) - startPose.getY(DistanceUnit.CM);
        double totalDistance = Math.hypot(deltaX, deltaY);

        double motionAngle = Math.atan2(deltaY, deltaX);

        double time = 0;
        double distanceTraveled = 0;

        while (distanceTraveled < totalDistance) {
            double velocity;

            Double lowerKey = accelMap.floorKey(distanceTraveled);
            Double higherKey = accelMap.ceilingKey(distanceTraveled);

            if (lowerKey != null && higherKey != null && !lowerKey.equals(higherKey)) {
                double t0 = accelMap.get(lowerKey);
                double t1 = accelMap.get(higherKey);

                double dtSegment = t1 - t0;
                double dd = higherKey - lowerKey;
                velocity = dd / dtSegment;
            } else {
                velocity = Constants.Pathing.SWERVE_MAX_VELOCITY;
            }

            distanceTraveled += velocity * dt;
            if (distanceTraveled > totalDistance) distanceTraveled = totalDistance;

            double x = startPose.getX(DistanceUnit.CM) + (distanceTraveled / totalDistance) * deltaX;
            double y = startPose.getY(DistanceUnit.CM) + (distanceTraveled / totalDistance) * deltaY;

            double fraction = distanceTraveled / totalDistance;

            double headingDiff = endPose.getHeading(AngleUnit.DEGREES) - startPose.getHeading(AngleUnit.DEGREES);
            headingDiff = ((headingDiff + 180) % 360) - 180;

            double heading = startPose.getHeading(AngleUnit.DEGREES) + fraction * headingDiff;
            heading = ((heading + 180) % 360) - 180;

            Pose2D currentPose = new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, heading);

            path.add(new TrajectoryPoint(currentPose, velocity, time));

            time += dt;
        }

        return path;
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

}
