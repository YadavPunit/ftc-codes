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
@Autonomous(name = "blue_auto_fl", group = "Auto")
public class blue_auto_fl extends LinearOpMode {

    /* ================= PEDRO ================= */
    private Follower follower;
    private Paths paths;

    /* ================= HARDWARE ================= */
    private DcMotorEx shooterLeft, shooterRight, turret;
    private DcMotor intake;
    private Servo hood, leftRamp, rightRamp;

    /* ================= SHOOTER (FROM Shooter_manual) ================= */
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 72.5;
    static final double SHOOTER_kI = 0.0;
    static final double SHOOTER_kD = 10.0;
    static final double SHOOTER_kF = 16.2;

    static final double SHOOTER_RPM = 2840;
    static final double HOOD_POS = 0.5;

    static final double RAMP_UP   = 0.57;
    static final double RAMP_DOWN = 0.30;

    /* ================= TURRET MECHANICS ================= */
    static final double GEAR_RATIO = 18.0;
    static final double TICKS_PER_DEG =
            (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    static final double MAX_TURRET_DEG = 125.0;

    /* ================= PEDRO TURRET PID ================= */
    static final double TURRET_kP = 0.012;
    static final double TURRET_kD = 0.001;
    static final double TURRET_MAX_POWER = 0.30;
    static final double TURRET_SLEW = 0.025;
    static final double TURRET_DEADBAND = 1.2;
    static final double TURRET_OFFSET_DEG = 3.8; // LEFT bias

    /* ================= BLUE GOAL ================= */
    static final double GOAL_X = 26;
    static final double GOAL_Y = 129;

    /* ================= STATE ================= */
    private double lastError = 0;
    private double lastPower = 0;

    @Override
    public void runOpMode() {

        /* ---------- Pedro ---------- */
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21, 124, Math.toRadians(315)));
        paths = new Paths(follower);

        /* ---------- Hardware ---------- */
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

        shooterLeft.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterRight.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hood.setPosition(HOOD_POS);
        setRamp(false);

        waitForStart();
        if (isStopRequested()) return;

        intake.setPower(-0.7);
        setShooterRPM(SHOOTER_RPM);

        runPath(paths.Path1, 0.9);  shoot();
        runPath(paths.Path2, 0.9);
        runPath(paths.Path3, 0.9); shoot();
        runPath(paths.Path4, 0.9);
        runPath(paths.Path5, 0.9);  shoot();
        runPath(paths.Path6, 0.9);
        runPath(paths.Path7, 0.9);  shoot();
        runPath(paths.Path8,0.9);
        ;
    }

    /* ================= PEDRO TURRET ================= */
    private void updateTurret() {

        Pose p = follower.getPose();

        double fieldAngle =
                Math.toDegrees(Math.atan2(GOAL_Y - p.getY(), GOAL_X - p.getX()));

        double targetDeg =
                normalize(fieldAngle - Math.toDegrees(p.getHeading()) + TURRET_OFFSET_DEG);

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
    private void shoot() {

        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 700) {
            follower.update();
            updateTurret();
        }

        setRamp(true);
        sleep(1400);
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

        public PathChain Path1, Path2, Path3, Path4, Path5,
                Path6, Path7, Path8, Path9, Path10, Path11;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.061, 122.946),

                                    new Pose(48.463, 94.742)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.463, 94.742),
                                    new Pose(59.344, 83.414),
                                    new Pose(41.286, 83.215),
                                    new Pose(23, 82.592)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.466, 83.592),

                                    new Pose(49.530, 94.148)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(49.530, 94.148),
                                    new Pose(47.112, 52.930),
                                    new Pose(54.653, 60.251),
                                    new Pose(18.046, 58.737)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.046, 59.737),

                                    new Pose(48.679, 94.054)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.679, 94.054),
                                    new Pose(51.048, 31.792),
                                    new Pose(54.198, 32.190),
                                    new Pose(18.774, 33.459)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.774, 35.459),

                                    new Pose(49.396, 94.388)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49.396, 94.388),

                                    new Pose(37.234, 71.776)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

        }
    }
        }

