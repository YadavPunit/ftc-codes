package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blue_back_right3", group = "Examples")
public class example_auto3 extends LinearOpMode {

    private Follower follower;
    private Paths paths;

    private DcMotor shooting;
    private DcMotor shooting1;
    private DcMotor intake;
    private Servo ramp;
    private Servo hood;

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(89.495, 7.750, Math.toRadians(90)));

        shooting  = hardwareMap.get(DcMotor.class, "shooting");
        shooting1 = hardwareMap.get(DcMotor.class, "shooting1");
        intake    = hardwareMap.get(DcMotor.class, "intake");
        ramp      = hardwareMap.get(Servo.class, "ramp1");
        hood      = hardwareMap.get(Servo.class, "hood");

        shooting.setDirection(DcMotor.Direction.REVERSE);
        shooting.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooting1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        shooting.setPower(0);
        shooting1.setPower(0);
        intake.setPower(0);

        // 🔹 RAMP CLOSED → SPIN SHOOTER TO 0.1 IN 1s
        ramp.setPosition(0.92);
        rampShooterToPointOne();

        paths = new Paths(follower);

        telemetry.addLine("Auto Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runPath(paths.Path1);
        visionShootSequence();

        intake.setPower(-0.8);
        runPath(paths.Path2);
        sleep(800);
        intake.setPower(-0.35);

        runPath(paths.Path3);
        visionShootSequence();

        intake.setPower(-0.8);
        runPath(paths.Path4);
        sleep(800);
        intake.setPower(-0.35);

        runPath(paths.Path5);
        visionShootSequence();

        intake.setPower(-0.8);
        runPath(paths.Path6);
        sleep(800);
        intake.setPower(-0.35);

        runPath(paths.Path7);
        visionShootSequence();

        runPath(paths.Path8);

        telemetry.addLine("Auto Complete");
        telemetry.update();
    }

    // ================= RUN PATH =================
    private void runPath(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    // ================= RAMP → SHOOTER RAMP-UP =================
    private void rampShooterToPointOne() {
        double targetPower = 0.9;
        int steps = 20;
        double stepPower = targetPower / steps;
        int stepTime = 1000 / steps; // 1 second total

        for (int i = 1; i <= steps && opModeIsActive(); i++) {
            double p = stepPower * i;
            shooting.setPower(p);
            shooting1.setPower(p);
            sleep(stepTime);
        }
    }

    // ================= VISION SHOOT SEQUENCE =================
    private void visionShootSequence() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        double tx = result.getTx();
        double ty = result.getTy();
        double tyAdjusted = ty - 5; // TY OFFSET (unchanged)

        double hoodPos = map(tx, 4.44, 18.14, 0.8, 0.3);
        hoodPos = clip(hoodPos, 0.3, 0.8);
        hood.setPosition(hoodPos);

        double shooterPower = map(Math.abs(tx), 4.44, 18.14, 0.58, 0.89);
        shooterPower = clip(shooterPower, 0.58, 0.89);
        shooting.setPower(shooterPower);
        shooting1.setPower(shooterPower);

        sleep(1000);
        intake.setPower(-1);

        sleep(1000);
        ramp.setPosition(0.6);

        sleep(3000);

        shooting.setPower(0);
        shooting1.setPower(0);
        intake.setPower(-0.35);
        ramp.setPosition(0.92);
    }

    // ================= UTIL =================
    private double map(double x, double inMin, double inMax,
                       double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin)
                / (inMax - inMin) + outMin;
    }

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ================= PATH DEFINITIONS =================
    public static class Paths {

        public PathChain Path1, Path2, Path3, Path4,
                Path5, Path6, Path7, Path8;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(89.720, 8.075),
                            new Pose(72.000, 72.224)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(45))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(72.000, 72.224),
                            new Pose(76.037, 92.636),
                            new Pose(103.710, 92.411)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(45),
                            Math.toRadians(4))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(103.710, 92.411),
                            new Pose(73.794, 79.178)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(4),
                            Math.toRadians(41))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(73.794, 79.178),
                            new Pose(70.654, 66.617),
                            new Pose(103.486, 69.533)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(41),
                            Math.toRadians(2))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(103.486, 69.533),
                            new Pose(75.364, 81.645)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(2),
                            Math.toRadians(41))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(75.364, 81.645),
                            new Pose(69.084, 37.009),
                            new Pose(103.607, 46.430)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(41),
                            Math.toRadians(2))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(103.607, 46.430),
                            new Pose(74.243, 80.523)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(2),
                            Math.toRadians(41))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(74.243, 80.523),
                            new Pose(74.916, 63.925)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(41),
                            Math.toRadians(90))
                    .build();
        }
    }
}
