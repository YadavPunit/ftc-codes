package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "auto_code1", group = "Examples")
public class ExampleAuto extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private final Pose startPose = new Pose(56.000, 8.00, Math.toRadians(90));

    private Path scorePreload;
    private Path scoreshoot;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower.setStartingPose(startPose);

        Servo myServo = hardwareMap.get(Servo.class, "shoot");
        myServo.setPosition(0.0);

        // Path 1
        scorePreload = new Path(
                new BezierCurve(
                        new Pose(56.000, 8.000),
                        new Pose(63.701, 38.131),
                        new Pose(18.841, 35.664)
                )
        );
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));

        // Path 2
        scoreshoot = new Path(
                new BezierCurve(
                        new Pose(18.841, 35.664),
                        new Pose(54.280, 30.729),
                        new Pose(56.000, 11.500)
                )
        );
        scoreshoot.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(100));

        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        opmodeTimer.resetTimer();

        // === PATH 1 ===
        follower.followPath(scorePreload);
        boolean servoActivated = false;
        sleep(1500);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            double currentX = follower.getPose().getX();

            if (!servoActivated && currentX > 75) {
                myServo.setPosition(0.97);
                servoActivated = true;
                telemetry.addLine("Servo activated at midway!");
            }

            telemetry.addData("Path", "1 - Moving to score");
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }

        myServo.setPosition(0.47);
        sleep(1000); // Optional wait before second path

        // === PATH 2 ===
        follower.followPath(scoreshoot);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Path", "2 - Returning to start");
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }

        telemetry.addLine("All paths complete!");
        telemetry.update();
    }
}
