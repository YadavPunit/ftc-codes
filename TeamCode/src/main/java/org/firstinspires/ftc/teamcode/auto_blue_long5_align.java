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

@Autonomous(name = "auto_blue_long5_align", group = "Auto")
public class auto_blue_long5_align extends LinearOpMode {

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
                new Pose(21, 124, Math.toRadians(315)) // 🔴 CHANGE START POSE HERE
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
        hood.setPosition(0.74);

        paths = new Paths(follower);
        opmodeTimer = new Timer();

        telemetry.addLine("Auto Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        limelight.start();
        sleep(500);

        // ================= AUTO SEQUENCE =================

        setShooterRPM(3330);
        intake.setPower(0.85);


        runPath(paths.Path1, 0.63);   // fast
        intake.setPower(0.8);
        sleep(1000);

        left_ramp.setPosition(0.55);
        right_ramp.setPosition(0.45);

        sleep(400);
        setShooterRPM(3480);

        sleep(2000);


        setShooterRPM(0);
        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);

        sleep(500);

        intake.setPower(0.85);



        runPath(paths.Path2, 0.6);   // slow for accuracy


        setShooterRPM(3370);




        runPath(paths.Path3, 0.7);
        intake.setPower(0.8);
        left_ramp.setPosition(0.54);
        right_ramp.setPosition(0.46);
        sleep(400);
        setShooterRPM(3585);

        sleep(1500);



        setShooterRPM(0);
        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);

        setShooterRPM(3380);


        runPath(paths.Path4, 0.6);

        sleep(500);

        runPath(paths.Path5, 0.6);
        intake.setPower(0.8);
        left_ramp.setPosition(0.55);
        right_ramp.setPosition(0.45);
        sleep(400);
        setShooterRPM(3520);

        sleep(3000);

        setShooterRPM(0);
        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);

        runPath(paths.Path6, 0.6);

        setShooterRPM(3380);

        sleep(500);

        runPath(paths.Path7, 0.6);
        intake.setPower(0.8);
        left_ramp.setPosition(0.55);
        right_ramp.setPosition(0.45);
        sleep(400);
        setShooterRPM(3520);

        sleep(3000);




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

        public PathChain Path1, Path2, Path3,Path4,Path5,Path6,Path7;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.308, 123.813),

                                    new Pose(70.579, 82.785)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(135))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(70.579, 72.785),
                                    new Pose(74.692, 82.766),
                                    new Pose(22.140, 84.019)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();


            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.140, 84.019),

                                    new Pose(70.879, 82.673)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(146))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(70.879, 72.673),
                                    new Pose(70.051, 58.991),
                                    new Pose(17, 60.112)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(146), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.607, 60.112),

                                    new Pose(70.449, 82.654)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(146))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(70.897, 82.748),
                                    new Pose(79.318, 30.514),
                                    new Pose(17.794, 35.869)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(146), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.794, 35.869),

                                    new Pose(70.860, 82.729)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(146))

                    .build();
        }
    }
}
