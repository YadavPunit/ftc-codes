package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blue_bl2", group = "Auto")
public class blue_bl2 extends LinearOpMode {

    /* ================= PEDRO ================= */
    private Follower follower;
    private Paths paths;

    /* ================= HARDWARE ================= */
    private DcMotorEx shooterL, shooterR;
    private DcMotor intake;
    private DcMotorEx turret;
    private Servo hood, left_ramp, right_ramp;

    /* ================= LIMELIGHT ================= */
    private Limelight3A limelight;

    /* ================= SHOOTER ================= */
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 98;
    static final double SHOOTER_kI = 0;
    static final double SHOOTER_kD = 14;
    static final double SHOOTER_kF = 18;

    /* ================= TURRET LIMITS ================= */
    static final int TURRET_MIN_TICKS = -175;
    static final int TURRET_MAX_TICKS = 175;

    /* ================= TURRET LIMELIGHT CONTROL ================= */
    static final double TURRET_KP_NEAR = 0.026;
    static final double TURRET_KP_FAR  = 0.098;

    static final double TURRET_MAX_POWER = 0.6;
    static final double TURRET_DEADBAND = 0.4;

    /* ================= LIVE TURRET STATE ================= */
    private boolean turretTrackingEnabled = false;
    private double turretTyOffset = 0;
    private double turretKP = TURRET_KP_NEAR;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(54.206, 6.1, Math.toRadians(180)));

        shooterL = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterR = hardwareMap.get(DcMotorEx.class, "right_shooting");
        turret   = hardwareMap.get(DcMotorEx.class, "turret");
        intake   = hardwareMap.get(DcMotor.class, "intake");

        hood = hardwareMap.get(Servo.class, "hood");
        left_ramp = hardwareMap.get(Servo.class, "left_ramp");
        right_ramp = hardwareMap.get(Servo.class, "right_ramp");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

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

        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);
        hood.setPosition(0.34);

        paths = new Paths(follower);

        telemetry.addLine("Auto Ready (Turret Tracking Enabled)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        /* ================= AUTO ================= */
        intake.setPower(0.45);
        setShooterRPM(3790);
        sleep(1000);

        runPath(paths.Path1, 0.65);
        alignAndShoot(3745, true, 3, TURRET_KP_NEAR);

        runPath(paths.Path2, 0.65);
        runPath(paths.Path3, 0.55);
        alignAndShoot(3745, true, 0, TURRET_KP_NEAR);

        runPath(paths.Path4, 0.65);
        hood.setPosition(0.56);
        setShooterRPM(2700);
        runPath(paths.Path5, 0.55);
        alignAndShoot(2780, true, 0, TURRET_KP_FAR);

        runPath(paths.Path6, 0.65);
        runPath(paths.Path7, 0.55);
        alignAndShoot(2780, true, 0, TURRET_KP_FAR);

        runPath(paths.Path8, 0.65);

        turretTrackingEnabled = false;
        turret.setPower(0);

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }

    /* ================= SHOOT ================= */
    private void alignAndShoot(double rpm, boolean turretAlign,
                               double tyOffset, double kp) {

        turretTrackingEnabled = turretAlign;
        turretTyOffset = tyOffset;
        turretKP = kp;

        left_ramp.setPosition(0.55);
        right_ramp.setPosition(0.45);
        sleep(1500);

        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);

        turretTrackingEnabled = false;
        turret.setPower(0);
    }

    /* ================= LIVE TURRET UPDATE (KEY ADDITION) ================= */
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

        double ty = result.getTy() + turretTyOffset;

        if (Math.abs(ty) < TURRET_DEADBAND) {
            turret.setPower(0);
            return;
        }

        double power = ty * turretKP;
        power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));

        int pos = turret.getCurrentPosition();

        if ((pos <= TURRET_MIN_TICKS && power < 0) ||
                (pos >= TURRET_MAX_TICKS && power > 0)) {
            power = 0;
        }

        turret.setPower(power);

        telemetry.addData("TY", ty);
        telemetry.addData("Turret Pos", pos);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    /* ================= PATH ================= */
    private void runPath(PathChain path, double speed) {

        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTurretTracking(); // ✅ turret corrects while driving
        }

        follower.setMaxPower(1.0);
    }

    /* ================= SHOOTER ================= */
    private void setShooterRPM(double rpm) {
        double ticks = (rpm / 60.0) * TICKS_PER_REV;
        shooterL.setVelocity(ticks);
        shooterR.setVelocity(ticks);
    }

    /* ================= PATHS ================= */
    public static class Paths {

        public PathChain Path1, Path2, Path3, Path4, Path5,
                Path6, Path7, Path8;

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
