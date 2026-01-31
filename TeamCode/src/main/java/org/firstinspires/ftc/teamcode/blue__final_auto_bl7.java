package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "blue__final_auto_bl7", group = "Auto")
public class blue__final_auto_bl7 extends LinearOpMode {

    /* ================= CORE ================= */
    private Follower follower;
    private Paths paths;

    /* ================= LIMELIGHT ================= */
    private Limelight3A limelight;

    /* ================= HARDWARE ================= */
    private DcMotorEx shooterLeft, shooterRight, turret;
    private DcMotor intake;
    private Servo hood, leftRamp, rightRamp;

    /* ================= MODE FLAGS ================= */
    private boolean pedroTurretActive = true;
    private boolean limelightTurretActive = false;

    /* ================= FIELD ================= */
    static final double GOAL_X = 29;
    static final double GOAL_Y = 129;

    /* ================= CONSTANTS ================= */
    static final double TICKS_PER_REV = 28.0;
    static final double GEAR_RATIO = 18.0;
    static final double TICKS_PER_DEG = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    static final double RAMP_UP = 0.57;
    static final double RAMP_DOWN = 0.30;

    /* ================= SHOOTER ================= */
    static final double RPM_TOLERANCE = 75;
    static final long RPM_TIMEOUT_MS = 1500;

    /* ================= TURRET ================= */
    static final int TURRET_MIN = -175;
    static final int TURRET_MAX = 175;

    static final double PEDRO_TURRET_kP = 0.018;

    static final double LL_kP = 0.016;
    static final double LL_kD = 0.002;
    static final double LL_DEADBAND = 0.6;
    static final double LL_MAX_POWER = 0.35;

    private double lastTurretErr = 0;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(54.206, 6.1, Math.toRadians(180)));
        paths = new Paths(follower);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterRight = hardwareMap.get(DcMotorEx.class, "right_shooting");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        intake = hardwareMap.get(DcMotor.class, "intake");

        hood = hardwareMap.get(Servo.class, "hood");
        leftRamp = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPower(-0.7);
        setRamp(false);

        waitForStart();
        if (isStopRequested()) return;

        /* ================= AUTO FLOW ================= */

        runPath(paths.Path1, 0.7);
        shoot(3820, 0.24, -2.0, -0.45, 1500);

        runPath(paths.Path2, 0.7);
        runPath(paths.Path3, 0.7);
        shoot(3820, 0.24, -1.4, -0.45, 1500);

        runPath(paths.Path4, 0.7);
        runPath(paths.Path5, 0.7);
        shoot(2750, 0.50,  4.0, -0.6, 1200);

        runPath(paths.Path6, 0.7);
        runPath(paths.Path7, 0.7);
        shoot(2750, 0.50, -4.0, -0.7, 1200);

        runPath(paths.Path8, 0.7);
    }

    /* ================= SHOOT ================= */
    private void shoot(
            double rpm,
            double hoodPos,
            double limelightOffset,
            double intakePower,
            long feedMs
    ) {

        pedroTurretActive = false;
        limelightTurretActive = true;

        setShooterRPM(rpm);
        hood.setPosition(hoodPos);
        intake.setPower(intakePower);

        waitForShooterRPM(rpm);

        setRamp(true);
        sleep(feedMs);
        setRamp(false);

        limelightTurretActive = false;
        pedroTurretActive = true;
    }

    /* ================= WAIT FOR RPM ================= */
    private void waitForShooterRPM(double targetRPM) {

        long start = System.currentTimeMillis();

        while (opModeIsActive()) {

            double rpmL = Math.abs(shooterLeft.getVelocity()) / TICKS_PER_REV * 60.0;
            double rpmR = Math.abs(shooterRight.getVelocity()) / TICKS_PER_REV * 60.0;
            double avg = (rpmL + rpmR) / 2.0;

            updateTurretLimelight(0);

            if (avg >= targetRPM - RPM_TOLERANCE) break;
            if (System.currentTimeMillis() - start > RPM_TIMEOUT_MS) break;
        }
    }

    /* ================= PATH ================= */
    private void runPath(PathChain path, double speed) {

        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTurretPedro();
        }
    }

    /* ================= PEDRO TURRET (GOAL TRACKING) ================= */
    private void updateTurretPedro() {

        if (!pedroTurretActive) return;

        Pose p = follower.getPose();

        double dx = GOAL_X - p.getX();
        double dy = GOAL_Y - p.getY();

        double targetHeading = Math.atan2(dy, dx);
        double errorRad = targetHeading - p.getHeading();

        while (errorRad > Math.PI)  errorRad -= 2 * Math.PI;
        while (errorRad < -Math.PI) errorRad += 2 * Math.PI;

        double errorDeg = Math.toDegrees(errorRad);
        double power = PEDRO_TURRET_kP * (errorDeg * TICKS_PER_DEG);

        int pos = turret.getCurrentPosition();
        if ((power > 0 && pos >= TURRET_MAX) ||
                (power < 0 && pos <= TURRET_MIN)) power = 0;

        turret.setPower(clamp(power, -0.45, 0.45));
    }

    /* ================= LIMELIGHT TURRET ================= */
    private void updateTurretLimelight(double offset) {

        if (!limelightTurretActive) return;

        LLResult ll = limelight.getLatestResult();
        if (ll == null || !ll.isValid()) {
            turret.setPower(0);
            return;
        }

        double error = ll.getTy() + offset;
        if (Math.abs(error) < LL_DEADBAND) {
            turret.setPower(0);
            return;
        }

        double d = error - lastTurretErr;
        lastTurretErr = error;

        double power = LL_kP * error + LL_kD * d;
        turret.setPower(clamp(power, -LL_MAX_POWER, LL_MAX_POWER));
    }

    /* ================= HELPERS ================= */
    private void setShooterRPM(double rpm) {
        double ticks = (rpm / 60.0) * TICKS_PER_REV;
        shooterLeft.setVelocity(ticks);
        shooterRight.setVelocity(ticks);
    }

    private void setRamp(boolean up) {
        double p = up ? RAMP_UP : RAMP_DOWN;
        leftRamp.setPosition(p);
        rightRamp.setPosition(1.0 - p);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /* ================= PATHS ================= */
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4,
                Path5, Path6, Path7, Path8;

        public Paths(Follower f) {

            Path1 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(54.207, 6.100),
                            new Pose(59.140, 14.467)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path2 = f.pathBuilder()
                    .addPath(new BezierCurve(new Pose(59.140, 14.467),
                            new Pose(44.434, 42.313),
                            new Pose(39.537, 35.238),
                            new Pose(18.589, 35.785)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path3 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(18.589, 35.785),
                            new Pose(59.056, 14.542)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = f.pathBuilder()
                    .addPath(new BezierCurve(new Pose(59.056, 14.542),
                            new Pose(48.799, 65.444),
                            new Pose(46.678, 60.304),
                            new Pose(18.075, 60.065)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path5 = f.pathBuilder()
                    .addPath(new BezierCurve(new Pose(18.075, 60.065),
                            new Pose(38.360, 69.860),
                            new Pose(49.243, 85.953)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path6 = f.pathBuilder()
                    .addPath(new BezierCurve(new Pose(49.243, 85.953),
                            new Pose(41.215, 83.238),
                            new Pose(19.280, 84.318)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path7 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(19.280, 84.318),
                            new Pose(49.140, 85.850)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path8 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(49.140, 85.850),
                            new Pose(37.234, 71.776)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
        }
    }
}
