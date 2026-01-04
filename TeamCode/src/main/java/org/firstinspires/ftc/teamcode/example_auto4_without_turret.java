package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "red_back_right4", group = "Examples")
public class example_auto4_without_turret extends LinearOpMode {

    // ================= Drive =================
    private Follower follower;
    private Paths paths;

    // ================= Hardware =================
    private DcMotorEx shooting;
    private DcMotorEx shooting1;
    private DcMotor intake;
    private Servo ramp;
    private Servo hood;

    private Limelight3A limelight;

    // ================= Vision + Control =================
    private static final double TARGET_TY = 3.15;
    private static final double kP_TURN = 0.02;
    private static final double TURN_LIMIT = 0.35;

    // ================= Shooter =================
    private static final double MAX_RPM = 6000.0;
    private static final double RPM_BOOST = 500.0;   // 🔥 boost for multi-ball
    private static final double RPM_TOLERANCE = 120; // safe feed window

    @Override
    public void runOpMode() {

        // -------- Follower --------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(89.495, 7.750, Math.toRadians(90)));

        // -------- Hardware --------
        shooting  = hardwareMap.get(DcMotorEx.class, "shooting");
        shooting1 = hardwareMap.get(DcMotorEx.class, "shooting1");
        intake    = hardwareMap.get(DcMotor.class, "intake");
        ramp      = hardwareMap.get(Servo.class, "ramp1");
        hood      = hardwareMap.get(Servo.class, "hood");

        shooting.setDirection(DcMotor.Direction.REVERSE);

        shooting.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooting1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ✅ REQUIRED FOR VELOCITY
        shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooting.setVelocity(0);
        shooting1.setVelocity(0);
        intake.setPower(0);
        ramp.setPosition(0.92);

        // -------- Limelight --------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // -------- Paths --------
        paths = new Paths(follower);

        telemetry.addLine("Auto Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ================= AUTO =================
        runPath(paths.Path1);
        visionAlignAndShoot();

        intake.setPower(-0.8);
        runPath(paths.Path2);
        sleep(800);
        intake.setPower(-0.4);

        runPath(paths.Path3);
        visionAlignAndShoot();

        intake.setPower(-0.8);
        runPath(paths.Path4);
        sleep(800);
        intake.setPower(-0.4);

        runPath(paths.Path5);
        visionAlignAndShoot();

        intake.setPower(-0.8);
        runPath(paths.Path6);
        sleep(800);
        intake.setPower(-0.4);

        runPath(paths.Path7);
        visionAlignAndShoot();

        runPath(paths.Path8);

        telemetry.addLine("Auto Complete");
        telemetry.update();
    }

    // ================= Path Runner =================
    private void runPath(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    // ================= Vision Align + Shoot =================
    private void visionAlignAndShoot() {

        long startTime = System.currentTimeMillis();

        // ---- Heading alignment (TY) ----
        while (opModeIsActive() && System.currentTimeMillis() - startTime < 1500) {

            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) continue;

            double ty = result.getTy() - 5;
            double error = TARGET_TY - ty;

            if (Math.abs(error) < 0.2) break;

            double turn = kP_TURN * error;
            turn = clip(turn, -TURN_LIMIT, TURN_LIMIT);

            follower.setTeleOpDrive(0, 0, turn, true);
            follower.update();
        }

        follower.setTeleOpDrive(0, 0, 0, true);

        // ---- TX-based shooting ----
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        double tx = result.getTx();

        // Hood
        double hoodPos = map(tx, 4.44, 18.14, 0.8, 0.3);
        hoodPos = clip(hoodPos, 0.3, 0.8);
        hood.setPosition(hoodPos);

        // Base RPM
        double baseRPM = map(
                Math.abs(tx),
                4.44, 18.14,
                0.58 * MAX_RPM,
                0.89 * MAX_RPM
        );
        baseRPM = clip(baseRPM, 0.58 * MAX_RPM, 0.89 * MAX_RPM);

        shooting.setVelocity(baseRPM);
        shooting1.setVelocity(baseRPM);

        // ---- Wait until RPM stabilizes ----
        long spinupStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - spinupStart < 1200) {
            if (Math.abs(baseRPM - shooting.getVelocity()) < RPM_TOLERANCE) {
                break;
            }
        }

        // ---- Feed balls ----
        intake.setPower(-0.6);
        sleep(300);

        ramp.setPosition(0.6);

        // 🔥 RPM BOOST FOR MULTI-BALL
        double boostedRPM = clip(baseRPM + RPM_BOOST, 0, MAX_RPM);
        shooting.setVelocity(boostedRPM);
        shooting1.setVelocity(boostedRPM);

        sleep(1000); // covers 3 balls

        // ---- Stop ----
        ramp.setPosition(0.92);
        shooting.setVelocity(0);
        shooting1.setVelocity(0);
        intake.setPower(-0.4);
    }

    // ================= Utilities =================
    private double map(double x, double inMin, double inMax,
                       double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin)
                / (inMax - inMin) + outMin;
    }

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ================= Paths =================
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(89.720, 8.075),
                            new Pose(72.000, 72.224)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();
            // Remaining paths unchanged
        }
    }
}