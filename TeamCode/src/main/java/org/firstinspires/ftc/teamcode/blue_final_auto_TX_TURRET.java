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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blue_final_auto_TX_TURRET", group = "Auto")
public class blue_final_auto_TX_TURRET extends LinearOpMode {

    /* ================= PEDRO ================= */
    private Follower follower;
    private Paths paths;

    /* ================= HARDWARE ================= */
    private DcMotorEx shooterL, shooterR, turret;
    private DcMotor intake;
    private Servo hood, leftRamp, rightRamp;
    private Limelight3A limelight;

    /* ================= SHOOTER ================= */
    static final double TICKS_PER_REV = 28.0;
    static final double SHOOTER_kP = 125;
    static final double SHOOTER_kI = 0;
    static final double SHOOTER_kD = 35;
    static final double SHOOTER_kF = 20;
    private double targetVelocity = 0;

    /* ================= TURRET TX TRACKING ================= */
    private static final int RIGHT_LIMIT = 175;
    private static final int LEFT_LIMIT  = -175;

    private static final double TURRET_KP = 0.035;
    private static final double TURRET_MAX_POWER = 0.4;
    private static final double TURRET_DEADBAND = 1.2;

    private int turretEncoderOffset = 0;
    private boolean turretTrackingEnabled = true;

    /* ================= SHOOT TIMING ================= */
    private boolean shootingActive = false;
    private long shootStartTime = 0;
    static final long SHOOT_TIME_MS = 1800;

    /* ================= INTAKE ================= */
    private double currentShootFeedPower = 0.45;
    private static final double INTAKE_COLLECT_POWER = 0.65;

    /* ================= STATE MACHINE ================= */
    enum AutoState {
        PATH1, SHOOT1,
        PATH2, PATH3, SHOOT2,
        PATH4, PATH5, SHOOT3,
        PATH6, PATH7, SHOOT4,
        DONE
    }

    AutoState state = AutoState.PATH1;

    /* ================= INIT ================= */
    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(54.206, 6.2, Math.toRadians(180)));

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

        shooterL.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterR.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hood.setPosition(0.25);
        rampDown();

        paths = new Paths(follower);

        waitForStart();
        if (isStopRequested()) return;

        turretEncoderOffset = turret.getCurrentPosition();

        intake.setPower(INTAKE_COLLECT_POWER);
        setShooterRPM(3710);

        followPathWithPower(paths.Path1, 0.83);

        while (opModeIsActive() && state != AutoState.DONE) {

            follower.update();
            updateShooter();
            updateTurretTracking();

            if (!shootingActive) {
                rampDown();
                intake.setPower(INTAKE_COLLECT_POWER);
            }

            switch (state) {

                case PATH1:
                    if (!follower.isBusy()) state = AutoState.SHOOT1;
                    break;

                case SHOOT1:
                    handleShooting(() -> {
                        followPathWithPower(paths.Path2, 0.85);
                        state = AutoState.PATH2;
                    });
                    break;

                case PATH2:
                    if (!follower.isBusy()) {
                        followPathWithPower(paths.Path3, 0.85);
                        state = AutoState.PATH3;
                    }
                    break;

                case PATH3:
                    if (!follower.isBusy()) state = AutoState.SHOOT2;
                    break;

                case SHOOT2:
                    handleShooting(() -> {
                        followPathWithPower(paths.Path4, 0.75);
                        state = AutoState.PATH4;
                    });
                    break;

                case PATH4:
                    hood.setPosition(0.5);
                    setShooterRPM(2740);
                    if (!follower.isBusy()) {
                        followPathWithPower(paths.Path5, 0.65);
                        state = AutoState.PATH5;
                    }
                    break;

                case PATH5:
                    if (!follower.isBusy()) state = AutoState.SHOOT3;
                    break;

                case SHOOT3:
                    handleShooting(() -> {
                        followPathWithPower(paths.Path6, 0.7);
                        state = AutoState.PATH6;
                    });
                    break;

                case PATH6:
                    if (!follower.isBusy()) {
                        followPathWithPower(paths.Path7, 0.7);
                        state = AutoState.PATH7;
                    }
                    break;

                case PATH7:
                    if (!follower.isBusy()) state = AutoState.SHOOT4;
                    break;

                case SHOOT4:
                    handleShooting(() -> state = AutoState.DONE);
                    break;
            }
        }
    }

    /* ================= TURRET TX TRACKING ================= */
    private void updateTurretTracking() {

        if (!turretTrackingEnabled) {
            turret.setPower(0);
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            turret.setPower(0);
            return;
        }

        double tx = result.getTx();

        if (Math.abs(tx) < TURRET_DEADBAND) {
            turret.setPower(0);
            return;
        }

        double power = tx * TURRET_KP;
        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        int relativePos = turret.getCurrentPosition() - turretEncoderOffset;

        if ((relativePos <= LEFT_LIMIT && power < 0) ||
                (relativePos >= RIGHT_LIMIT && power > 0)) {
            power = 0;
        }

        turret.setPower(power);
    }

    /* ================= HELPERS ================= */
    private void followPathWithPower(PathChain path, double power) {
        follower.setMaxPower(power);
        follower.followPath(path);
    }

    /* ================= SHOOT HANDLER ================= */
    private void handleShooting(Runnable onFinish) {

        if (!shootingActive && shooterAtSpeed()) {
            shootingActive = true;
            shootStartTime = System.currentTimeMillis();
            rampUp();
            intake.setPower(currentShootFeedPower);
        }

        if (shootingActive &&
                System.currentTimeMillis() - shootStartTime >= SHOOT_TIME_MS) {

            shootingActive = false;
            rampDown();
            intake.setPower(INTAKE_COLLECT_POWER);
            onFinish.run();
        }
    }

    /* ================= SHOOTER ================= */
    private void setShooterRPM(double rpm) {
        targetVelocity = (rpm / 60.0) * TICKS_PER_REV;
    }

    private void updateShooter() {
        shooterL.setVelocity(targetVelocity);
        shooterR.setVelocity(targetVelocity);
    }

    private boolean shooterAtSpeed() {
        return Math.abs(shooterL.getVelocity() - targetVelocity) < 60;
    }

    /* ================= RAMPS ================= */
    private void rampUp() {
        leftRamp.setPosition(0.55);
        rightRamp.setPosition(0.45);
    }

    private void rampDown() {
        leftRamp.setPosition(0.3);
        rightRamp.setPosition(0.7);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /* ================= PATHS ================= */
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

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
        }
    }
}
