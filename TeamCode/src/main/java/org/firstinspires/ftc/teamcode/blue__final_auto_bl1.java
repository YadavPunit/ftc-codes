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
@Autonomous(name = "blue__final_auto_bl1", group = "Auto")
public class blue__final_auto_bl1 extends LinearOpMode {

    /* ================= PEDRO ================= */
    private Follower follower;
    private Paths paths;

    /* ================= HARDWARE ================= */
    private DcMotorEx shooterLeft, shooterRight, turret;
    private DcMotor intake;
    private Servo hood, leftRamp, rightRamp;

    /* ================= SHOOTER ================= */
    static final double TICKS_PER_REV = 28.0;

    /* ===== CLOSE PIDF ===== */
    static final double FAR_kP = 72;
    static final double FAR_kI = 0.0;
    static final double FAR_kD = 10.0;
    static final double FAR_kF = 16.2;

    /* ===== FAR PIDF ===== */
    static final double CLOSE_kP = 96.0;
    static final double CLOSE_kI = 0.0;
    static final double CLOSE_kD = 19.0;
    static final double CLOSE_kF = 19.5;

    static final double RAMP_UP   = 0.57;
    static final double RAMP_DOWN = 0.30;

    /* ================= DYNAMIC SHOOTING (BY Y) ================= */
    static final double CLOSE_Y_THRESHOLD = 47.0;

    // CLOSE
    static final double CLOSE_TURRET_OFFSET = -14;
    static final double CLOSE_SHOOTER_RPM   = 3640;
    static final double CLOSE_HOOD_POS      = 0.25;

    // FAR
    static final double FAR_TURRET_OFFSET   = 3;
    static final double FAR_SHOOTER_RPM     = 2750;
    static final double FAR_HOOD_POS        = 0.5;

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
    static final double GOAL_X = 13;
    static final double GOAL_Y = 129;

    /* ================= STATE ================= */
    private double lastError = 0;
    private double lastPower = 0;

    private enum ShooterMode { CLOSE, FAR }
    private ShooterMode currentShooterMode = null;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(54.206, 6.1, Math.toRadians(180)));
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
        sleep(800);
        intake.setPower(-0.7);

        runPath(paths.Path1, 0.9);
        intake.setPower(-0.5);
        shoot(1500);
        intake.setPower(-0.7);

        runPath(paths.Path2, 0.9);
        runPath(paths.Path3, 0.9);
        intake.setPower(-0.5);
        shoot(1500);
        intake.setPower(-0.7);
        runPath(paths.Path4, 0.9);
        runPath(paths.Path5, 0.9); shoot(1200);
        runPath(paths.Path6, 0.9);
        runPath(paths.Path7, 0.9); shoot(1200);
        runPath(paths.Path8, 0.9);
    }

    /* ================= PEDRO TURRET ================= */
    private void updateTurret() {

        Pose p = follower.getPose();

        double fieldAngle =
                Math.toDegrees(Math.atan2(GOAL_Y - p.getY(), GOAL_X - p.getX()));

        double offsetDeg = isCloseShot()
                ? FAR_TURRET_OFFSET
                : CLOSE_TURRET_OFFSET;

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

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(41.486, 95.411),
                            new Pose(37.234, 71.776)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
        }
    }
}
