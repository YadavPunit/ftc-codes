package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "auto_blue_long3_align", group = "Auto")
public class auto_blue_long3_align extends LinearOpMode {

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;
    private Timer opmodeTimer;

    // ================= HARDWARE =================
    private DcMotorEx shooterL, shooterR;
    private DcMotor intake;
    private DcMotorEx turret;
    private Servo hood, left_ramp, right_ramp;
    private Limelight3A limelight;

    // ================= CONSTANTS =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 60.0;
    static final double SHOOTER_kI = 0.0;
    static final double SHOOTER_kD = 6.0;
    static final double SHOOTER_kF = 12.0;

    // ===== TURRET LIMITS (±75°) =====
    static final int TURRET_MIN_TICKS = -189;
    static final int TURRET_MAX_TICKS =  189;

    // ===== TURRET PD =====
    static final double TURRET_kP = 0.0197;
    static final double TURRET_kD = 0.001;

    static final double TURRET_DEADBAND = 0.18;
    static final double TURRET_MAX_POWER = 0.35;
    static final double TURRET_SLEW = 0.04;
    static final double ERROR_ALPHA = 0.20;
    static final double TY_OFFSET = 0.0;

    // ================= STATE =================
    private double filteredError = 0;
    private double lastError = 0;
    private double lastTurretPower = 0;
    private boolean turretAlignEnabled = false;

    // ================= INIT =================
    @Override
    public void runOpMode() {

        // ---------- FOLLOWER ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(54, 7, Math.toRadians(90)));

        // ---------- HARDWARE ----------
        shooterL = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterR = hardwareMap.get(DcMotorEx.class, "right_shooting");
        turret   = hardwareMap.get(DcMotorEx.class, "turret");
        intake   = hardwareMap.get(DcMotor.class, "intake");

        hood = hardwareMap.get(Servo.class, "hood");
        left_ramp = hardwareMap.get(Servo.class, "left_ramp");
        right_ramp = hardwareMap.get(Servo.class, "right_ramp");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterR.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------- SERVO POS ----------
        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);
        hood.setPosition(0.28);

        paths = new Paths(follower);
        opmodeTimer = new Timer();

        telemetry.addLine("Auto Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        limelight.start();
        sleep(500);

        // ================= AUTO =================

        setShooterRPM(3790);
        runPath(paths.Path1);

        intake.setPower(0.4);
        enableTurretAlign();   // ✅ START ALIGN

        sleep(1200);

        left_ramp.setPosition(0.6);
        right_ramp.setPosition(0.4);

        sleep(1200);

        disableTurretAlign();  // ❌ STOP ALIGN
        setShooterRPM(0);

        runPath(paths.Path2);

        setShooterRPM(3790);
        enableTurretAlign();

        runPath(paths.Path3);

        disableTurretAlign();
        setShooterRPM(0);

        telemetry.addLine("Auto Complete");
        telemetry.update();
    }

    // ================= TURRET ALIGN =================
    private void enableTurretAlign() {
        turretAlignEnabled = true;
        updateTurretAlign();
    }

    private void disableTurretAlign() {
        turretAlignEnabled = false;
        stopTurret();
    }

    private void updateTurretAlign() {

        if (!turretAlignEnabled) return;

        LLResult ll = limelight.getLatestResult();
        if (ll == null || !ll.isValid()) {
            stopTurret();
            return;
        }

        double rawError = ll.getTy() - TY_OFFSET;
        filteredError =
                ERROR_ALPHA * rawError +
                        (1.0 - ERROR_ALPHA) * filteredError;

        if (Math.abs(filteredError) < TURRET_DEADBAND) {
            stopTurret();
            return;
        }

        double derivative = filteredError - lastError;
        lastError = filteredError;

        double power =
                (TURRET_kP * filteredError) +
                        (TURRET_kD * derivative);

        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        double delta = clamp(power - lastTurretPower,
                -TURRET_SLEW, TURRET_SLEW);
        power = lastTurretPower + delta;
        lastTurretPower = power;

        int pos = turret.getCurrentPosition();
        if ((power > 0 && pos >= TURRET_MAX_TICKS) ||
                (power < 0 && pos <= TURRET_MIN_TICKS)) {
            stopTurret();
            return;
        }

        turret.setPower(power);
    }

    private void stopTurret() {
        turret.setPower(0);
        lastTurretPower = 0;
        lastError = 0;
        filteredError = 0;
    }

    // ================= SHOOTER RPM =================
    private void setShooterRPM(double rpm) {
        double ticks = (rpm / 60.0) * TICKS_PER_REV;
        shooterL.setVelocity(ticks);
        shooterR.setVelocity(ticks);
    }

    // ================= PATH RUNNER =================
    private void runPath(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTurretAlign(); // 🔁 keep aligning during motion
            telemetry.update();
        }
    }

    // ================= HELPERS =================
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ================= PATHS =================
    public static class Paths {

        public PathChain Path1, Path2, Path3;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54, 7),
                            new Pose(65.196, 15.813)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(108))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(65.196, 15.813),
                            new Pose(60.299, 38.005),
                            new Pose(24.822, 35.935)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(108),
                            Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(20, 35.935),
                            new Pose(57, 20)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(135))
                    .build();
        }
    }
}
