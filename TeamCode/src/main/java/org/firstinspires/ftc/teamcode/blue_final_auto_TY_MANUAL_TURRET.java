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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blue_final_auto_TY_MANUAL_TURRET", group = "Auto")
public class blue_final_auto_TY_MANUAL_TURRET extends LinearOpMode {

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;

    // ================= HARDWARE =================
    private DcMotorEx shooterL, shooterR, turret;
    private DcMotor intake;
    private Servo hood, leftRamp, rightRamp;
    private Limelight3A limelight;

    // ================= SHOOTER =================
    static final double TICKS_PER_REV = 28.0;
    private double targetVelocity = 0;

    // ================= TURRET =================
    private static final double TICKS_PER_DEGREE = 1.4;

    private static final int RIGHT_LIMIT = 400;
    private static final int LEFT_LIMIT  = -400;

    private static final double KP_TY = 0.035;
    private static final double KP_DEGREE = 0.015;
    private static final double MAX_POWER = 0.4;
    private static final double DEADBAND = 1.2;

    private boolean turretAutoAlignEnabled = true;
    private boolean manualDegreeEnabled = false;

    private int turretEncoderOffset = 0;
    private int manualTargetTicks = 0;

    // ================= INTAKE =================
    private static final double INTAKE_COLLECT_POWER = 0.65;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(54.206, 6.2, Math.toRadians(180)));

        shooterL = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterR = hardwareMap.get(DcMotorEx.class, "right_shooting");
        turret   = hardwareMap.get(DcMotorEx.class, "turret");
        intake   = hardwareMap.get(DcMotor.class, "intake");

        hood = hardwareMap.get(Servo.class, "hood");
        leftRamp = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hood.setPosition(0.25);
        rampDown();

        paths = new Paths(follower);

        waitForStart();
        if (isStopRequested()) return;

        turretEncoderOffset = turret.getCurrentPosition();

        intake.setPower(INTAKE_COLLECT_POWER);
        setShooterRPM(3710);
        setManualTurretDegrees(-68);
        sleep(1000);

        enableTurretAutoAlign();
        runPath(paths.Path1, 0.83);
        shootTimed(2000, 0.45);

        setManualTurretDegrees(-68);
        runPath(paths.Path2, 0.85);
        runPath(paths.Path3, 0.85);

        enableTurretAutoAlign();
        shootTimed(1800, 0.45);

        hood.setPosition(0.5);
        setShooterRPM(2740);

        runPath(paths.Path4, 0.75);
        runPath(paths.Path5, 0.65);
        shootTimed(1800, 0.6);

        runPath(paths.Path6, 0.7);
        runPath(paths.Path7, 0.7);
        shootTimed(1500, 0.6);

        intake.setPower(0);
        setShooterRPM(0);
    }

    // ================= TURRET MODES =================
    private void enableTurretAutoAlign() {
        turretAutoAlignEnabled = true;
        manualDegreeEnabled = false;
    }

    private void setManualTurretDegrees(double degrees) {
        manualDegreeEnabled = true;
        turretAutoAlignEnabled = false;
        manualTargetTicks = (int)(degrees * TICKS_PER_DEGREE);
    }

    // ================= TURRET UPDATE =================
    private void updateTurret() {

        int currentTicks = turret.getCurrentPosition() - turretEncoderOffset;

        if (manualDegreeEnabled) {
            int error = manualTargetTicks - currentTicks;
            if (Math.abs(error) < 3) {
                turret.setPower(0);
                return;
            }

            double power = clamp(error * KP_DEGREE, -MAX_POWER, MAX_POWER);
            turret.setPower(power);
            return;
        }

        if (!turretAutoAlignEnabled) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            turret.setPower(0);
            return;
        }

        double ty = result.getTy();
        if (Math.abs(ty) < DEADBAND) {
            turret.setPower(0);
            return;
        }

        turret.setPower(clamp(ty * KP_TY, -MAX_POWER, MAX_POWER));
    }

    // ================= PATH =================
    private void runPath(PathChain path, double speed) {

        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTurret();
            updateShooter();
        }

        follower.setMaxPower(1.0);
    }

    // ================= SHOOT (FIXED) =================
    private void shootTimed(int timeMs, double feed) {

        intake.setPower(feed);
        rampUp();

        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeMs) {
            updateTurret();
            updateShooter();   // ✅ FIX — keeps RPM stable
        }

        rampDown();
        intake.setPower(INTAKE_COLLECT_POWER);
    }

    // ================= SHOOTER =================
    private void setShooterRPM(double rpm) {
        targetVelocity = (rpm / 60.0) * TICKS_PER_REV;
    }

    private void updateShooter() {
        shooterL.setVelocity(targetVelocity);
        shooterR.setVelocity(targetVelocity);
    }

    // ================= RAMPS =================
    private void rampUp() {
        leftRamp.setPosition(0.55);
        rightRamp.setPosition(0.45);
    }

    private void rampDown() {
        leftRamp.setPosition(0.3);
        rightRamp.setPosition(0.7);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ================= PATHS =================
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.206, 6.1),
                            new Pose(59.140, 14.467)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(59.140, 14.467),
                            new Pose(59.393, 37.668),
                            new Pose(61.682, 37.458),
                            new Pose(16.168, 36.925)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(16.168, 36.925),
                            new Pose(59.178, 14.411)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(59.178, 14.411),
                            new Pose(61.930, 65.972),
                            new Pose(54.509, 64.093),
                            new Pose(17.991, 62.374)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(17.991, 62.374),
                            new Pose(50.836, 65.047),
                            new Pose(41.701, 95.308)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(41.701, 95.308),
                            new Pose(43.206, 83.542),
                            new Pose(47.019, 83.374),
                            new Pose(18.935, 83.963)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.935, 83.963),
                            new Pose(41.486, 95.411)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}
