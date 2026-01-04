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

@Autonomous(name = "red_short", group = "Examples")
public class red_back_right extends LinearOpMode {

    private Follower follower;
    private Paths paths;
    private Timer opmodeTimer;

    private DcMotor shooting;
    private DcMotor shooting1;

    private Servo hood;

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
        shooting.setDirection(DcMotor.Direction.REVERSE);

        shooting1 = hardwareMap.get(DcMotor.class, "shooting1");
        shooting1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ramp = hardwareMap.get(Servo.class, "ramp1");

        hood = hardwareMap.get(Servo.class, "hood");

        shooting.setPower(0);
        intake.setPower(0);
        hood.setPosition(0.3);
        ramp.setPosition(0.92);

        paths = new Paths(follower);
        opmodeTimer = new Timer();

        telemetry.addLine("Auto Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        opmodeTimer.resetTimer();

        shooting.setPower(0.91);
        shooting1.setPower(0.91);


        runPath(paths.Path1);
        intake.setPower(-0.6);
        sleep(500);
        ramp.setPosition(0.6);
        sleep(300);
        shooting.setPower(0.95);
        shooting1.setPower(0.95);
        sleep(300);
        shooting.setPower(1);
        shooting1.setPower(1);
        sleep(2500);
        intake.setPower(0);
        ramp.setPosition(0.92);
        intake.setPower(-0.6);

        runPath(paths.Path2);

        sleep(500);

        intake.setPower(0);
        shooting.setPower(0.91);
        shooting1.setPower(0.91);

        runPath(paths.Path3);
        ramp.setPosition(0.6);
        sleep(2500);
        shooting.setPower(0.95);
        shooting1.setPower(0.95);
        sleep(300);
        shooting.setPower(1);
        shooting1.setPower(1);
        hood.setPosition(0.3);
        sleep(2000);
        intake.setPower(-0.6);

        runPath(paths.Path4);
        intake.setPower(0);
        shooting.setPower(0.91);
        shooting1.setPower(0.91);

        runPath(paths.Path5);
        ramp.setPosition(0.6);
        sleep(2500);
        shooting.setPower(0.95);
        shooting1.setPower(0.95);
        sleep(300);
        shooting.setPower(1);
        shooting1.setPower(1);
        hood.setPosition(0.3);
        sleep(2000);
        intake.setPower(-0.6);

        runPath(paths.Path6);
        intake.setPower(0);
        shooting.setPower(0.91);
        shooting1.setPower(0.91);

        runPath(paths.Path7);
        ramp.setPosition(0.6);
        sleep(2500);
        shooting.setPower(0.95);
        shooting1.setPower(0.95);
        sleep(300);
        shooting.setPower(1);
        shooting1.setPower(1);
        hood.setPosition(0.3);
        sleep(2000);
        intake.setPower(-0.6);



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

            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.056, 8.075), new Pose(66.617, 17.271))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                    .build();
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(66.617, 17.271),
                                    new Pose(68.86, 34.766),
                                    new Pose(43.065, 34.093)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(177))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.065, 34.093), new Pose(66.617, 17.047))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(177), Math.toRadians(117))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(66.617, 17.047),
                                    new Pose(85.907, 51.140),
                                    new Pose(45.308, 57.645)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(117), Math.toRadians(176))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.308, 57.645), new Pose(66.617, 16.822))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(176), Math.toRadians(117))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(66.617, 16.822),
                                    new Pose(89.047, 75.140),
                                    new Pose(40.598, 82.991)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(117), Math.toRadians(176))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(66.617, 16.822), new Pose(66.617, 17.271))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(117), Math.toRadians(117))
                    .build();
        }
    }
}





