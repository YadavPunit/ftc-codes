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

@Autonomous(name = "blue_final_auto_longshoot_var", group = "Auto")
public class blue_final_auto_longshoot_var extends LinearOpMode {

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;

    // ================= HARDWARE =================
    private DcMotorEx shooterL, shooterR;
    private DcMotorEx turret;
    private DcMotor intake;
    private Servo hood, leftRamp, rightRamp;
    private Limelight3A limelight;

    // ================= SHOOTER =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 90;
    static final double SHOOTER_kI = 0;
    static final double SHOOTER_kD = 15;
    static final double SHOOTER_kF = 18;

    // ================= TURRET =================
    static final double TURRET_kP = 0.088;
    static final double TURRET_kD = 0.011;
    static final double TURRET_DEADBAND = 0.15;
    static final double TURRET_MAX_POWER = 0.85;
    static final double TURRET_SLEW = 0.038;
    static final double TURRET_ALIGN_DAMP = 0.2;

    static final int TURRET_MIN_TICKS = -430;
    static final int TURRET_MAX_TICKS = 430;

    static final double TURRET_LL_OFFSET_DEG = 0.7;
    static final double TICKS_PER_DEGREE = 3.36;

    private double lastTurretError = 0;
    private double lastTurretPower = 0;
    private boolean turretAlignEnabled = true;

    // ================= INIT =================
    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(54.206, 7.327, Math.toRadians(90)));

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

        shooterL.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterR.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRamp.setPosition(0.3);
        rightRamp.setPosition(0.7);
        hood.setPosition(0.25);

        paths = new Paths(follower);

        waitForStart();
        if (isStopRequested()) return;

        // ================= AUTO SEQUENCE =================

        intake.setPower(0.75);
        applyShooterMappingFromLimelight();
        sleep(700);

        runPath(paths.Path1, 0.83);
        moveTurretDegrees(60);
        sleep(800);
        alignAndShoot(1900, 0.45);

        runPath(paths.Path2, 0.65);
        intake.setPower(0.65);
        applyShooterMappingFromLimelight();

        runPath(paths.Path3, 0.67);
        moveTurretDegrees(60);
        sleep(800);
        alignAndShoot(1900, 0.45);

        runPath(paths.Path4, 0.65);
        intake.setPower(0.65);
        applyShooterMappingFromLimelight();

        runPath(paths.Path5, 0.75);
        moveTurretDegrees(39);
        sleep(800);
        alignAndShoot(1400, 0.62);

        runPath(paths.Path6, 0.65);
        intake.setPower(0.65);
        applyShooterMappingFromLimelight();

        runPath(paths.Path7, 0.75);
        moveTurretDegrees(37);
        sleep(800);
        alignAndShoot(1400, 0.62);

        leftRamp.setPosition(0.3);
        rightRamp.setPosition(0.7);
        sleep(800);
    }

    // ================= SHOOTER MAPPING (FROM TELEOP) =================
    private void applyShooterMappingFromLimelight() {

        LLResult ll = limelight.getLatestResult();
        if (ll == null || !ll.isValid()) return;

        double tx = ll.getTx();
        double rpm, hoodPos;

        if (tx >= 15.77) {
            rpm = 3680; hoodPos = 0.25;
        } else if (tx >= 14.18) {
            rpm = map(tx, 14.18, 15.77, 3360, 3630);
            hoodPos = map(tx, 14.18, 15.77, 0.28, 0.25);
        } else if (tx >= 13.18) {
            rpm = map(tx, 13.18, 14.18, 3200, 3350);
            hoodPos = map(tx, 13.18, 14.18, 0.39, 0.28);
        } else if (tx >= 11.92) {
            rpm = map(tx, 11.92, 13.18, 3050, 3200);
            hoodPos = map(tx, 11.92, 13.18, 0.41, 0.39);
        } else if (tx >= 9.96) {
            rpm = map(tx, 9.96, 11.92, 2900, 3050);
            hoodPos = map(tx, 9.96, 11.92, 0.45, 0.41);
        } else if (tx >= 8.08) {
            rpm = map(tx, 8.08, 9.96, 2800, 2900);
            hoodPos = map(tx, 8.08, 9.96, 0.49, 0.45);
        } else if (tx >= 6.42) {
            rpm = map(tx, 6.42, 8.08, 2660, 2800);
            hoodPos = map(tx, 6.42, 8.08, 0.52, 0.49);
        } else if (tx >= 3.50) {
            rpm = map(tx, 3.50, 6.42, 2600, 2660);
            hoodPos = map(tx, 3.50, 6.42, 0.60, 0.52);
        } else if (tx >= 1.14) {
            rpm = map(tx, 1.14, 3.50, 2540, 2600);
            hoodPos = map(tx, 1.14, 3.50, 0.75, 0.60);
        } else {
            rpm = 2470; hoodPos = 0.89;
        }

        setShooterRPM(rpm);
        hood.setPosition(hoodPos);
    }

    // ================= TURRET =================
    private void updateTurret(boolean enable) {

        if (!enable) {
            turret.setPower(0);
            lastTurretError = 0;
            lastTurretPower = 0;
            return;
        }

        LLResult ll = limelight.getLatestResult();
        if (ll == null || !ll.isValid()) return;

        double error = ll.getTy() + TURRET_LL_OFFSET_DEG;
        double power = 0;

        if (Math.abs(error) > TURRET_DEADBAND) {
            double d = error - lastTurretError;
            lastTurretError = error;
            power = (TURRET_kP * error) + (TURRET_kD * d);
        }

        power *= TURRET_ALIGN_DAMP;
        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        double delta = clamp(power - lastTurretPower, -TURRET_SLEW, TURRET_SLEW);
        power = lastTurretPower + delta;
        lastTurretPower = power;

        int pos = turret.getCurrentPosition();
        if ((power > 0 && pos >= TURRET_MAX_TICKS) ||
                (power < 0 && pos <= TURRET_MIN_TICKS)) power = 0;

        turret.setPower(power);
    }

    private void moveTurretDegrees(double deg) {

        turretAlignEnabled = false;

        int ticks = (int) (deg * TICKS_PER_DEGREE);
        ticks = (int) clamp(ticks, TURRET_MIN_TICKS, TURRET_MAX_TICKS);

        turret.setTargetPosition(ticks);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.6);

        while (opModeIsActive() && turret.isBusy()) idle();

        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretAlignEnabled = true;
    }

    // ================= SHOOT =================
    private void alignAndShoot(int timeMs, double feed) {

        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeMs) {

            applyShooterMappingFromLimelight();
            updateTurret(turretAlignEnabled);

            intake.setPower(feed);
            leftRamp.setPosition(0.55);
            rightRamp.setPosition(0.45);

            sleep(40);
        }

        leftRamp.setPosition(0.3);
        rightRamp.setPosition(0.7);
        intake.setPower(0.65);
    }

    // ================= PATH =================
    private void runPath(PathChain path, double speed) {

        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTurret(turretAlignEnabled);
        }
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

    private double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);
    }

    // ================= PATHS =================
    public static class Paths {

        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.206, 7.327),
                            new Pose(59.140, 14.467)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
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
