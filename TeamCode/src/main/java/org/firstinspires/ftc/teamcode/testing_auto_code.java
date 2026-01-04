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

@Autonomous(name = "testing auto code", group = "Examples")
public class testing_auto_code extends LinearOpMode {

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

        shooting.setPower(0.95);
        shooting1.setPower(0.95);
        sleep(1000);

        runPath(paths.Path1);

        intake.setPower(-0.7);
        sleep(800);

        ramp.setPosition(0.7);
        sleep(2000);

        shooting.setPower(0);
        shooting1.setPower(0);
        intake.setPower(0);
        ramp.setPosition(0.92);
        sleep(400);


        runPath(paths.Path2);

        intake.setPower(-1);

        runPath(paths.Path3);

        sleep(400);

        intake.setPower(0);

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

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.009, 84.785),
                                    new Pose(85.682, 32.299),
                                    new Pose(90, 35.215)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(92.636, 35.215),
                                    new Pose(97.505, 35.215))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

        }
    }
}
