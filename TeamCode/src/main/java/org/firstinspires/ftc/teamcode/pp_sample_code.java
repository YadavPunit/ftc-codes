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

@Autonomous(name = "blue_back_right_final", group = "Examples")
public class pp_sample_code extends LinearOpMode {

    private Follower follower;
    private Paths paths;
    private Timer opmodeTimer;
    private DcMotor shooting1;
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

        shooting1 = hardwareMap.get(DcMotor.class, "shooting1");
        shooting1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooting.setDirection(DcMotor.Direction.REVERSE);

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

        shooting.setPower(0.71);
        shooting1.setPower(0.71);
        sleep(1000);

        runPath(paths.Path1);

        intake.setPower(-0.7);
        sleep(800);

        ramp.setPosition(0.6);
        sleep(2000);

        shooting.setPower(0);
        shooting1.setPower(0);
        intake.setPower(0);



        ramp.setPosition(0.92);
        sleep(400);
        intake.setPower(-1);

        runPath(paths.Path2);

        intake.setPower(0);
        shooting.setPower(0.71);
        shooting1.setPower(0.71);

        runPath(paths.Path3);

        sleep(600);
        intake.setPower(-1);
        sleep(600);
        ramp.setPosition(0.6);
        sleep(2000);
        ramp.setPosition(0.92);
        sleep(200);
        shooting.setPower(0);
        shooting1.setPower(0);

        runPath(paths.Path4);

        sleep(400);
        intake.setPower(0);
        shooting.setPower(0.71);
        shooting1.setPower(0.71);

        runPath(paths.Path5);

        intake.setPower(-1);
        sleep(600);
        ramp.setPosition(0.6);
        sleep(1000);
        ramp.setPosition(0.92);
        sleep(400);
        shooting.setPower(0);
        shooting1.setPower(0);

        runPath(paths.Path6);

        sleep(400);
        intake.setPower(0);
        shooting.setPower(0.71);
        shooting1.setPower(0.71);

        runPath(paths.Path7);

        intake.setPower(-1);
        sleep(400);
        ramp.setPosition(0.6);
        sleep(700);
        shooting.setPower(0);
        shooting1.setPower(0);
        ramp.setPosition(0.92);
        sleep(200);

        runPath(paths.Path8);

        sleep(100);
        intake.setPower(0);
        shooting.setPower(0);
        shooting1.setPower(0);

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

        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(89.495, 7.750),
                                    new Pose(85.009, 84.785)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.009, 84.785),
                                    new Pose(83.439, 29.607),
                                    new Pose(110.355, 37.215)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(110.355, 37.215),
                                    new Pose(85.009, 84.561)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.009, 84.561),
                                    new Pose(95.103, 55.178),
                                    new Pose(110.131, 61.664)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(110.131, 59.664),
                                    new Pose(85.009, 84.785)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.009, 84.785),
                                    new Pose(109.234, 85.664)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(109.234, 85.664),
                                    new Pose(85.009, 84.785)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.009, 84.785),
                                    new Pose(91.514, 52.262)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();
        }
    }
}
