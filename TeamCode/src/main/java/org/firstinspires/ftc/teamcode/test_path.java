package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "test_path", group = "Pedro")
public class test_path extends OpMode {

    private Follower follower;
    private Paths paths;
    private int pathState = 0;

    @Override
    public void init() {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants
                .createFollower(hardwareMap);

        // Start pose MUST match Path1 start
        follower.setPose(new Pose(56.000, 8.000, Math.toRadians(90)));

        paths = new Paths(follower);
    }

    @Override
    public void loop() {

        follower.update();

        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);
                pathState++;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    pathState++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    pathState++;
                }
                break;

            case 4:
                // ✅ Correct way to stop Pedro follower
                follower.breakFollowing();
                break;
        }
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }

    // ------------------------------------------------------------------
    // PATH DEFINITIONS (UNCHANGED)
    // ------------------------------------------------------------------
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56.000, 8.000),
                            new Pose(59.439, 40.374),
                            new Pose(17.720, 35.664)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(17.720, 35.664),
                            new Pose(61.009, 38.355),
                            new Pose(68.636, 16.374)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(68.636, 16.374),
                            new Pose(73.570, 63.028),
                            new Pose(17.047, 59.439)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(17.047, 59.439),
                            new Pose(97.000, 9.000)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}
