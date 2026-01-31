package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "red_final_auto_br", group = "Auto")
public class red_final_auto_br extends LinearOpMode {

    /* ================= PEDRO ================= */
    private Follower follower;
    private Paths paths;

    /* ================= HARDWARE ================= */
    private DcMotorEx shooterLeft, shooterRight, turret;
    private DcMotor intake;
    private Servo hood, leftRamp, rightRamp;

    /* ================= SHOOTER ================= */
    static final double TICKS_PER_REV = 28.0;

    /* ===== FAR PIDF ===== */
    static final double CLOSE_kP = 72;
    static final double CLOSE_kI = 0.0;
    static final double CLOSE_kD = 10.0;
    static final double CLOSE_kF = 16.2;

    /* ===== CLOSE PIDF ===== */
    static final double FAR_kP = 95.0;
    static final double FAR_kI = 0.0;
    static final double FAR_kD = 8;
    static final double FAR_kF = 19;

    static final double RAMP_UP   = 0.57;
    static final double RAMP_DOWN = 0.30;

    /* ================= DYNAMIC SHOOTING (BY Y) ================= */
    static final double CLOSE_Y_THRESHOLD = 47.0;

    // CLOSE
    public static double FAR_TURRET_OFFSET = -4;
    public static double FAR_SHOOTER_RPM   = 3780;
    public static double FAR_HOOD_POS      = 0.25;

    // FAR
    public static double CLOSE_TURRET_OFFSET   = -4;
    public static double CLOSE_SHOOTER_RPM     = 2840;
    public static double CLOSE_HOOD_POS        = 0.5;

    /* ================= TURRET MECHANICS ================= */
    static final double GEAR_RATIO = 18.0;
    static final double TICKS_PER_DEG =
            (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    static final double MAX_TURRET_DEG = 125.0;

    /* ================= PEDRO TURRET PID ================= */
    static final double TURRET_kP = 0.012;
    static final double TURRET_kD = 0.001;
    static final double TURRET_MAX_POWER = 0.3;
    static final double TURRET_SLEW = 0.025;
    static final double TURRET_DEADBAND = 1.2;

    /* ================= BLUE GOAL ================= */
    static final double GOAL_X = 115;
    static final double GOAL_Y = 129;

    /* ================= STATE ================= */
    private double lastError = 0;
    private double lastPower = 0;

    private enum ShooterMode { CLOSE, FAR }
    private ShooterMode currentShooterMode = null;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(86.854, 6.123, Math.toRadians(0)));
        paths = new Paths(follower);

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterRight = hardwareMap.get(DcMotorEx.class, "right_shooting");
        turret       = hardwareMap.get(DcMotorEx.class, "turret");
        intake       = hardwareMap.get(DcMotor.class, "intake");

        hood      = hardwareMap.get(Servo.class, "hood");
        leftRamp  = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.REVERSE);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setRamp(false);
        hood.setPosition(0.25);

        waitForStart();
        if (isStopRequested()) return;

        updateShooterForDistance();
        intake.setPower(-0.72);
        sleep(900);


        runPath(paths.Path1, 0.8);
        sleep(500);
        intake.setPower(-0.55);
        shoot(1500);
        intake.setPower(-0.72);

        runPath(paths.Path2, 0.7);
        runPath(paths.Path3, 1);
        intake.setPower(-0.55);
        shoot(1500);
        intake.setPower(-0.72);

        runPath(paths.Path4, 0.7);
        runPath(paths.Path5, 1); shoot(1200);
        runPath(paths.Path6, 0.7);
        runPath(paths.Path7, 1); shoot(1200);
        runPath(paths.Path8, 0.5);
    }

    /* ================= PEDRO TURRET ================= */
    private void updateTurret() {

        Pose p = follower.getPose();

        double fieldAngle =
                Math.toDegrees(Math.atan2(GOAL_Y - p.getY(), GOAL_X - p.getX()));

        // ✅ FIXED LOGIC
        double offsetDeg = isCloseShot()
                ? CLOSE_TURRET_OFFSET
                : FAR_TURRET_OFFSET;

        double targetDeg =
                normalize(fieldAngle - Math.toDegrees(p.getHeading()) + offsetDeg);

        targetDeg = clamp(targetDeg, -MAX_TURRET_DEG, MAX_TURRET_DEG);

        double currentDeg = turret.getCurrentPosition() / TICKS_PER_DEG;
        double error = targetDeg - currentDeg;

        if (Math.abs(error) < TURRET_DEADBAND) {
            turret.setPower(0);
            lastError = error;
            return;
        }

        double d = error - lastError;
        lastError = error;

        double power = TURRET_kP * error + TURRET_kD * d;
        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        double delta = clamp(power - lastPower, -TURRET_SLEW, TURRET_SLEW);
        power = lastPower + delta;
        lastPower = power;

        turret.setPower(power);
    }

    /* ================= SHOOT ================= */
    private void shoot(int feedtime) {

        updateShooterForDistance();

        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 700) {
            follower.update();
            updateTurret();
        }

        setRamp(true);
        sleep(feedtime);
        setRamp(false);
    }

    /* ================= PATH ================= */
    private void runPath(PathChain path, double speed) {

        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTurret();
        }
    }

    /* ================= HELPERS ================= */
    private boolean isCloseShot() {
        // ✅ CORRECT RULE:
        // Y > threshold → CLOSE
        // Y <= threshold → FAR
        return follower.getPose().getY() > CLOSE_Y_THRESHOLD;
    }

    private void updateShooterForDistance() {

        ShooterMode desiredMode = isCloseShot()
                ? ShooterMode.CLOSE
                : ShooterMode.FAR;

        if (desiredMode != currentShooterMode) {

            if (desiredMode == ShooterMode.CLOSE) {
                shooterLeft.setVelocityPIDFCoefficients(CLOSE_kP, CLOSE_kI, CLOSE_kD, CLOSE_kF);
                shooterRight.setVelocityPIDFCoefficients(CLOSE_kP, CLOSE_kI, CLOSE_kD, CLOSE_kF);
            } else {
                shooterLeft.setVelocityPIDFCoefficients(FAR_kP, FAR_kI, FAR_kD, FAR_kF);
                shooterRight.setVelocityPIDFCoefficients(FAR_kP, FAR_kI, FAR_kD, FAR_kF);
            }

            currentShooterMode = desiredMode;
        }

        if (currentShooterMode == ShooterMode.CLOSE) {
            setShooterRPM(CLOSE_SHOOTER_RPM);
            hood.setPosition(CLOSE_HOOD_POS);
        } else {
            setShooterRPM(FAR_SHOOTER_RPM);
            hood.setPosition(FAR_HOOD_POS);
        }
    }

    private void setShooterRPM(double rpm) {
        double ticks = (rpm / 60.0) * TICKS_PER_REV;
        shooterLeft.setVelocity(ticks);
        shooterRight.setVelocity(ticks);
    }

    private void setRamp(boolean up) {
        double pos = up ? RAMP_UP : RAMP_DOWN;
        leftRamp.setPosition(pos);
        rightRamp.setPosition(1.0 - pos);
    }

    private double normalize(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /* ================= PATHS ================= */
    public static class Paths {

        public PathChain Path1, Path2, Path3, Path4,
                Path5, Path6, Path7, Path8;

        public Paths(Follower follower) {

                Path1 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(86.854, 6.123),

                                        new Pose(79.597, 24.718)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path2 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(79.597, 24.718),
                                        new Pose(79.997, 39.855),
                                        new Pose(100.406, 33.587),
                                        new Pose(125.298, 35.490)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(125.298, 35.490),

                                        new Pose(79.809, 24.394)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path4 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(79.809, 24.394),
                                        new Pose(85.569, 70.871),
                                        new Pose(100.123, 57.294),
                                        new Pose(125.693, 59.191)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path5 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(125.693, 59.191),

                                        new Pose(79.861, 24.304)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path6 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(79.861, 24.304),
                                        new Pose(87.354, 89.944),
                                        new Pose(93.971, 84.583),
                                        new Pose(121.069, 83.367)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path7 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(121.069, 83.367),

                                        new Pose(80.093, 24.353)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(80.093, 24.353),

                                    new Pose(107, 71.776)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();
        }
    }
}
