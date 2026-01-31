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
@Autonomous(name = "blue__final_auto_bl3", group = "Auto")
public class blue__final_auto_bl3 extends LinearOpMode {

    /* ================= PEDRO ================= */
    private Follower follower;
    private Paths paths;

    /* ================= HARDWARE ================= */
    private DcMotorEx shooterLeft, shooterRight, turret;
    private DcMotor intake;
    private Servo hood, leftRamp, rightRamp;

    /* ================= CONSTANTS ================= */
    static final double TICKS_PER_REV = 28.0;
    static final double GEAR_RATIO = 18.0;
    static final double TICKS_PER_DEG = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    static final double GOAL_X = 13;
    static final double GOAL_Y = 129;

    static final double RAMP_UP = 0.57;
    static final double RAMP_DOWN = 0.30;

    /* ================= SHOOTER PID (AUTO SWITCH) ================= */
    static final double Y_PID_SWITCH = 47.0;

    // LOW PID → CLOSE
    static final double CLOSE_kP = 72.5, CLOSE_kI = 0, CLOSE_kD = 10,  CLOSE_kF = 16.2;

    // HIGH PID → FAR
    static final double FAR_kP   = 78.5, FAR_kI   = 0, FAR_kD   = 10, FAR_kF   = 16.2;

    enum ShooterPIDMode { CLOSE, FAR }
    ShooterPIDMode currentPID = null;

    /* ================= TURRET PID ================= */
    static final double TURRET_kP = 0.012;
    static final double TURRET_kD = 0.001;
    static final double TURRET_MAX_POWER = 0.30;
    static final double TURRET_SLEW = 0.025;
    static final double TURRET_DEADBAND = 0.7;
    static final double MAX_TURRET_DEG = 125;

    double lastTurretErr = 0;
    double lastTurretPower = 0;

    /* ================= SHOT CONFIG ================= */
    static class ShotConfig {
        double rpm;
        double hood;
        double intakePower;
        double turretOffsetDeg;
        long spinUpMs;
        long feedMs;

        ShotConfig(double rpm, double hood, double intakePower,
                   double turretOffsetDeg, long spinUpMs, long feedMs) {
            this.rpm = rpm;
            this.hood = hood;
            this.intakePower = intakePower;
            this.turretOffsetDeg = turretOffsetDeg;
            this.spinUpMs = spinUpMs;
            this.feedMs = feedMs;
        }
    }

    ShotConfig activeShot = null;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(54.206, 6.1, Math.toRadians(180)));
        paths = new Paths(follower);

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterRight = hardwareMap.get(DcMotorEx.class, "right_shooting");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        intake = hardwareMap.get(DcMotor.class, "intake");

        hood = hardwareMap.get(Servo.class, "hood");
        leftRamp = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.REVERSE);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setRamp(false);

        waitForStart();
        if (isStopRequested()) return;

        /* ================= DEFINE SHOTS ================= */
        ShotConfig SHOT_1 = new ShotConfig(3740, 0.25, -0.5, -16.5, 700, 1500);
        ShotConfig SHOT_2 = new ShotConfig(3740, 0.25, -0.5, -11, 700, 1500);
        ShotConfig SHOT_3 = new ShotConfig(2880, 0.51, -0.7,  4, 700, 1200);

        /* ================= AUTO SEQUENCE ================= */
        intake.setPower(-0.7);
        setShooterRPM(3780);
        sleep(900);
        runPath(paths.Path1, 0.7); shoot(SHOT_1);
        intake.setPower(-0.7);
        runPath(paths.Path2, 0.7);
        runPath(paths.Path3, 0.7);
        shoot(SHOT_2);
        intake.setPower(-0.7);
        runPath(paths.Path4, 0.7);
        runPath(paths.Path5, 0.7); shoot(SHOT_3);
        runPath(paths.Path6, 0.7);
        runPath(paths.Path7, 0.7); shoot(SHOT_3);
        runPath(paths.Path8, 0.7);
    }

    /* ================= SHOOT ================= */
    private void shoot(ShotConfig shot) {

        activeShot = shot;

        applyShooterPID();                 // PID by Y
        setShooterRPM(shot.rpm);           // per-shot RPM
        hood.setPosition(shot.hood);// per-shot hood
        intake.setPower(shot.intakePower); // per-shot intake

        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < shot.spinUpMs) {
            follower.update();
            updateTurret();
        }

        setRamp(true);
        sleep(shot.feedMs);
        setRamp(false);

    }

    /* ================= TURRET ================= */
    private void updateTurret() {

        Pose p = follower.getPose();

        double fieldAngle = Math.toDegrees(
                Math.atan2(GOAL_Y - p.getY(), GOAL_X - p.getX())
        );

        double offset = (activeShot != null) ? activeShot.turretOffsetDeg : 0;

        double targetDeg = normalize(
                fieldAngle - Math.toDegrees(p.getHeading()) + offset
        );

        targetDeg = clamp(targetDeg, -MAX_TURRET_DEG, MAX_TURRET_DEG);

        double currentDeg = turret.getCurrentPosition() / TICKS_PER_DEG;
        double error = targetDeg - currentDeg;

        if (Math.abs(error) < TURRET_DEADBAND) {
            turret.setPower(0);
            lastTurretErr = error;
            return;
        }

        double d = error - lastTurretErr;
        lastTurretErr = error;

        double power = TURRET_kP * error + TURRET_kD * d;
        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        double delta = clamp(power - lastTurretPower, -TURRET_SLEW, TURRET_SLEW);
        power = lastTurretPower + delta;
        lastTurretPower = power;

        turret.setPower(power);
    }

    /* ================= PID SWITCH ================= */
    private void applyShooterPID() {

        ShooterPIDMode desired =
                (follower.getPose().getY() > Y_PID_SWITCH)
                        ? ShooterPIDMode.CLOSE   // CLOSE → LOW PID
                        : ShooterPIDMode.FAR;    // FAR → HIGH PID

        if (desired == currentPID) return;

        if (desired == ShooterPIDMode.CLOSE) {
            shooterLeft.setVelocityPIDFCoefficients(CLOSE_kP, CLOSE_kI, CLOSE_kD, CLOSE_kF);
            shooterRight.setVelocityPIDFCoefficients(CLOSE_kP, CLOSE_kI, CLOSE_kD, CLOSE_kF);
        } else {
            shooterLeft.setVelocityPIDFCoefficients(FAR_kP, FAR_kI, FAR_kD, FAR_kF);
            shooterRight.setVelocityPIDFCoefficients(FAR_kP, FAR_kI, FAR_kD, FAR_kF);
        }

        currentPID = desired;
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
        public PathChain Path1, Path2, Path3, Path4,
                Path5, Path6, Path7, Path8;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(54.207, 6.100), new Pose(59.140, 14.467)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(new Pose(59.140, 14.467),
                                    new Pose(44.434, 42.313),
                                    new Pose(39.537, 35.238),
                                    new Pose(18.589, 35.785)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(18.589, 35.785), new Pose(59.056, 14.542)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(new Pose(59.056, 14.542),
                                    new Pose(48.799, 65.444),
                                    new Pose(46.678, 60.304),
                                    new Pose(18.075, 60.065)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(new Pose(18.075, 60.065),
                                    new Pose(38.360, 69.860),
                                    new Pose(49.243, 85.953)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(new Pose(49.243, 85.953),
                                    new Pose(41.215, 83.238),
                                    new Pose(19.280, 84.318)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(19.280, 84.318), new Pose(49.140, 85.850)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(49.140, 85.850), new Pose(37.234, 71.776)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
        }
    }
}
