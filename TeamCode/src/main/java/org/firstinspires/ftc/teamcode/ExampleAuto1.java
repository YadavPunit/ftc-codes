package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blue_back_right1", group = "Examples")
public class ExampleAuto1 extends LinearOpMode {

    private Follower follower;
    private Paths paths;
    private Timer opmodeTimer;

    private DcMotor shooting;
    private Servo ramp;
    private DcMotor intake;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(
                new Pose(89.495, 7.750, Math.toRadians(90))
        );

        shooting = hardwareMap.get(DcMotor.class, "shooting");
        shooting.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ramp = hardwareMap.get(Servo.class,"ramp1");

        shooting.setPower(0);
        intake.setPower(0);

        paths = new Paths(follower);
        opmodeTimer = new Timer();

        telemetry.addLine("Auto Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        opmodeTimer.resetTimer();


        runPath(paths.Path1);
        sleep(1000);

        runPath(paths.Path2);
        sleep(1000);

        runPath(paths.Path3);
        sleep(1000);

        runPath(paths.Path4);
        sleep(1000);

        runPath(paths.Path5);
        sleep(1000);
        runPath(paths.Path6);
        sleep(1000);
        runPath(paths.Path7);
        sleep(1000);
        runPath(paths.Path8);
        sleep(1000);
        runPath(paths.Path9);
        sleep(1000);
        runPath(paths.Path10);
        sleep(1000);
        runPath(paths.Path10);
        sleep(1000);


        telemetry.addLine("Auto Complete");
        telemetry.update();
    }

    private void runPath(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading",
                    Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    // ================= PATH DEFINITIONS =================
    public static class Paths {

        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9,Path10,Path11;

        public Paths(Follower follower) {

            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.374, 8.075), new Pose(72.224, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.224, 72.000), new Pose(90.168, 83.439))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90.168, 83.439), new Pose(102.056, 83.664))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.056, 83.664), new Pose(72.449, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.449, 72.000), new Pose(90.168, 58.766))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90.168, 58.766), new Pose(101.383, 58.991))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.383, 58.991), new Pose(72.449, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.449, 72.000), new Pose(91.963, 34.766))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(91.963, 34.766), new Pose(101.383, 34.766))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.383, 34.766), new Pose(72.224, 71.776))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.224, 71.776), new Pose(72.000, 38.804))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(275))
                    .build();
        }
    }
}
