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

@Autonomous(name = "auto_blue_long4_align", group = "Auto")
public class auto_blue_long4_align extends LinearOpMode {

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

    // Shooter PIDF
    static final double SHOOTER_kP = 60.0;
    static final double SHOOTER_kI = 0.0;
    static final double SHOOTER_kD = 6.0;
    static final double SHOOTER_kF = 12.0;

    // ===== TURRET LIMITS (9:1 × 3.6, ±75°) =====
    static final int TURRET_MIN_TICKS = -200;
    static final int TURRET_MAX_TICKS =  200;

    // ===== TURRET PD =====
    static final double TURRET_kP = 0.02;
    static final double TURRET_kD = 0.002;

    static final double TURRET_DEADBAND = 0.1;
    static final double TURRET_MAX_POWER = 0.5;
    static final double TURRET_SLEW = 0.04;
    static final double ERROR_ALPHA = 0.20;
    static final double TY_OFFSET = -0.8;

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
        follower.setStartingPose(
                new Pose(54, 7, Math.toRadians(90)) // 🔴 CHANGE START POSE HERE
        );

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

        // ---------- SERVO DEFAULTS ----------
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

        // ================= AUTO SEQUENCE =================

        setShooterRPM(3760);
        enableTurretAlign();

        runPath(paths.Path1, 0.9);   // fast
        intake.setPower(0.5);
        sleep(1500);

        left_ramp.setPosition(0.6);
        right_ramp.setPosition(0.4);

        sleep(2000);


        setShooterRPM(0);
        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);

        intake.setPower(0.4);

        setShooterRPM(0);

        runPath(paths.Path2, 0.7);   // slow for accuracy


        setShooterRPM(3760);



        runPath(paths.Path3, 0.7);
        left_ramp.setPosition(0.6);
        right_ramp.setPosition(0.4);
        sleep(1500);



        setShooterRPM(0);
        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);

        setShooterRPM(3790);


        runPath(paths.Path4, 0.7);

        runPath(paths.Path5, 0.5);
        left_ramp.setPosition(0.6);
        right_ramp.setPosition(0.4);
        sleep(2500);

        disableTurretAlign();



        telemetry.addLine("Auto Complete");
        telemetry.update();
    }

    // ================= PATH WITH SPEED =================
    private void runPath(PathChain path, double speed) {

        speed = clamp(speed, 0.0, 1.0);
        follower.setMaxPower(speed);

        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTurretAlign(); // keep turret alive if enabled
            telemetry.addData("Path Speed", speed);
            telemetry.update();
        }

        follower.setMaxPower(1.0); // restore full speed
    }

    // ================= TURRET ALIGN =================
    private void enableTurretAlign() {
        turretAlignEnabled = true;
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

    // ================= HELPERS =================
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ================= PATHS =================
    public static class Paths {

        public PathChain Path1, Path2, Path3,Path4,Path5;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54, 7),
                            new Pose(65.196, 15.813)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(110))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(62.804, 40.822),
                                    new Pose(15, 35.551)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24.822, 35.935),
                            new Pose(57, 20)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(115))
                    .build();
            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.551, 20.103),
                                    new Pose(65.720, 64.374),
                                    new Pose(15.000, 60.336)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(128), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.963, 60.336),

                                    new Pose(57.645, 20.636)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))

                    .build();


        }
    }
}
