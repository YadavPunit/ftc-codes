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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blue_apex_code2_FINAL", group = "Auto")
public class blue_apex_code2_FINAL extends LinearOpMode {

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;

    // ================= HARDWARE =================
    private DcMotorEx shooterL, shooterR;
    private DcMotor intake;
    private DcMotorEx turret;
    private Servo hood, left_ramp, right_ramp;
    private Limelight3A limelight;

    // ================= SHOOTER =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 60;
    static final double SHOOTER_kI = 0;
    static final double SHOOTER_kD = 6;
    static final double SHOOTER_kF = 12;

    // ================= TURRET =================
    static final int TURRET_DIRECTION = -1;   // flip if wrong
    static final int TURRET_MIN_TICKS = -200;
    static final int TURRET_MAX_TICKS =  200;

    static final double HARD_DEADBAND = 0.20;
    static final double SOFT_DEADBAND = 0.60;

    static final double HOLD_POWER = 0.05;
    static final double MAX_POWER_FAR = 0.40;
    static final double KP_NEAR = 0.015;
    static final double KD_NEAR = 0.004;

    static final double ERROR_ALPHA = 0.25;
    static final double TURRET_SLEW = 0.025;
    static final double TY_OFFSET = -0.8;

    static final double TURRET_MAX_POWER = 0.40;

    // ================= LIMELIGHT =================
    static final long LL_READ_INTERVAL_MS = 50;
    private long lastLLReadTime = 0;

    // ================= STATE =================
    private boolean turretAlignEnabled = false;
    private double filteredError = 0;
    private double lastError = 0;
    private double lastTurretPower = 0;

    // ================= INIT =================
    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21, 124, Math.toRadians(315)));

        shooterL = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterR = hardwareMap.get(DcMotorEx.class, "right_shooting");
        turret   = hardwareMap.get(DcMotorEx.class, "turret");
        intake   = hardwareMap.get(DcMotor.class, "intake");

        hood = hardwareMap.get(Servo.class, "hood");
        left_ramp = hardwareMap.get(Servo.class, "left_ramp");
        right_ramp = hardwareMap.get(Servo.class, "right_ramp");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(20);

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterR.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);
        hood.setPosition(0.32);

        paths = new Paths(follower);

        telemetry.addLine("Auto Ready (FINAL)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        limelight.start();
        sleep(400);

        // ================= AUTO =================
        intake.setPower(0.8);

        runPath(paths.Path1, 0.65);

        alignAndShoot(3200);
        runPath(paths.Path2, 0.9);

        runPath(paths.Path3, 0.55);

        runPath(paths.Path4, 0.65);

        alignAndShoot(3200);
        runPath(paths.Path5, 0.9);

        runPath(paths.Path6, 0.55);

        runPath(paths.Path7, 0.65);

        alignAndShoot(3200);
        runPath(paths.Path8, 0.9);

        runPath(paths.Path9, 0.55);

        runPath(paths.Path10, 0.65);
        alignAndShoot(3200);

        runPath(paths.Path11,0.7);

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }

    // ================= ALIGN + SHOOT =================
    private void alignAndShoot(double rpm) {
        turretAlignEnabled = true;
        setShooterRPM(rpm);
        sleep(1500);

        left_ramp.setPosition(0.55);
        right_ramp.setPosition(0.45);
        sleep(1500);

        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);
        setShooterRPM(0);

        turretAlignEnabled = false;
        stopTurret();
    }

    // ================= PATH =================
    private void runPath(PathChain path, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTurretAlign();
        }

        follower.setMaxPower(1.0);
    }

    // ================= TURRET ALIGN (NO WOBBLE) =================
    private void updateTurretAlign() {

        if (!turretAlignEnabled) return;

        long now = System.currentTimeMillis();
        if (now - lastLLReadTime < LL_READ_INTERVAL_MS) return;
        lastLLReadTime = now;

        LLResult ll;
        try {
            ll = limelight.getLatestResult();
        } catch (Exception e) {
            stopTurret();
            return;
        }

        if (ll == null || !ll.isValid()) {
            stopTurret();
            return;
        }

        double rawError = (ll.getTy() - TY_OFFSET) * TURRET_DIRECTION;
        filteredError = ERROR_ALPHA * rawError + (1 - ERROR_ALPHA) * filteredError;

        double absErr = Math.abs(filteredError);

        if (absErr < HARD_DEADBAND) {
            stopTurret();
            return;
        }

        double power;

        if (absErr < SOFT_DEADBAND) {
            double derivative = filteredError - lastError;
            power = KP_NEAR * filteredError + KD_NEAR * derivative;
            power += Math.signum(filteredError) * HOLD_POWER;
        } else {
            power = Math.signum(filteredError) * MAX_POWER_FAR;
        }

        lastError = filteredError;

        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        double delta = clamp(power - lastTurretPower, -TURRET_SLEW, TURRET_SLEW);
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

    // ================= SHOOTER =================
    private void setShooterRPM(double rpm) {
        double ticks = (rpm / 60.0) * TICKS_PER_REV;
        shooterL.setVelocity(ticks);
        shooterR.setVelocity(ticks);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ================= PATHS =================
    public static class Paths {

        public PathChain Path1, Path2, Path3,Path4, Path5, Path6,Path7,Path8, Path9, Path10, Path11;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.664, 123.514),

                                    new Pose(46.355, 96.785)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(135))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.355, 96.785),

                                    new Pose(45.000, 84.346)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.000, 84.346),

                                    new Pose(22.907, 84.224)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.907, 84.224),

                                    new Pose(46.355, 96.776)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.355, 96.776),

                                    new Pose(45.000, 60.215)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.000, 60.215),

                                    new Pose(23.318, 60.243)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.318, 60.243),

                                    new Pose(46.561, 96.757)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.561, 96.757),

                                    new Pose(45.000, 36.234)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.000, 36.234),

                                    new Pose(23.318, 36.019)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.318, 36.019),

                                    new Pose(46.561, 96.935)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.561, 96.935),

                                    new Pose(28.178, 71.682)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))

                    .build();
        }
    }
}
