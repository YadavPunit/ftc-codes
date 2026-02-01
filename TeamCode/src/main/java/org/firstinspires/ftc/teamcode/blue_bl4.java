package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "blue_bl4_FINAL_NATIONALS", group = "Auto")
public class blue_bl4 extends LinearOpMode {

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
    static final double SHOOT_RPM = 2900;
    static final double RPM_READY = 2500;
    static final double HOOD_POS = 0.5;

    /* ================= TURRET MECHANICS (YOUR VALUES) ================= */
    static final double GEAR_RATIO = 18.0;
    static final double TICKS_PER_DEG =
            (TICKS_PER_REV * GEAR_RATIO) / 360.0;   // ≈1.4
    static final double MAX_TURRET_DEG = 125.0;

    static final int TURRET_LEFT_LIMIT  =
            (int) (-MAX_TURRET_DEG * TICKS_PER_DEG);
    static final int TURRET_RIGHT_LIMIT =
            (int) ( MAX_TURRET_DEG * TICKS_PER_DEG);

    /* ================= PEDRO TURRET ================= */
    static final double PEDRO_kP = 0.014;
    static final double PEDRO_kD = 0.0012;
    static final double PEDRO_MAX_POWER = 0.35;
    static final double PEDRO_DEADBAND_DEG = 1.2;
    static final double PEDRO_OFFSET_DEG = 5.0; // LEFT bias

    /* ================= LIMELIGHT TY TRACK ================= */
    public static double LL_TY_OFFSET = -5;
    public static double LL_DEADBAND = 0.6;
    public static double LL_TICKS_PER_TY = 8.0;
    public static double LL_kP = 0.05;
    public static double LL_MAX_POWER = 0.25;

    /* ================= BLUE GOAL ================= */
    static final double GOAL_X = 13;
    static final double GOAL_Y = 129;

    /* ================= STATE ================= */
    private boolean limelightOverride = false;
    private double lastError = 0;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21, 124, Math.toRadians(315)));
        paths = new Paths(follower);

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
        turret.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setVelocityPIDFCoefficients(60,0,6,12);
        shooterR.setVelocityPIDFCoefficients(60,0,6,12);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood.setPosition(HOOD_POS);
        leftRamp.setPosition(0.3);
        rightRamp.setPosition(0.7);

        waitForStart();
        if (isStopRequested()) return;

        intake.setPower(0.75);
        setShooterRPM(SHOOT_RPM);

        runPath(paths.Path1, 0.65);
        alignAndShoot();
        runPath(paths.Path2, 0.9);
        runPath(paths.Path3, 0.55);
        runPath(paths.Path4, 0.65);  alignAndShoot();
        runPath(paths.Path5, 0.9);
        runPath(paths.Path6, 0.55);
        runPath(paths.Path7, 0.65);  alignAndShoot();
        runPath(paths.Path8, 0.9);
        runPath(paths.Path9, 0.55);
        runPath(paths.Path10,0.65);  alignAndShoot();
        runPath(paths.Path11,0.65);
    }

    /* ================= PEDRO TURRET (FIELD-CENTRIC) ================= */
    private void updateTurretPedro() {

        Pose p = follower.getPose();

        double fieldAngle =
                Math.toDegrees(Math.atan2(GOAL_Y - p.getY(), GOAL_X - p.getX()));

        double targetDeg = normalize(
                fieldAngle - Math.toDegrees(p.getHeading()) + PEDRO_OFFSET_DEG
        );
        targetDeg = clamp(targetDeg, -MAX_TURRET_DEG, MAX_TURRET_DEG);

        double currentDeg = turret.getCurrentPosition() / TICKS_PER_DEG;
        double error = targetDeg - currentDeg;

        if (Math.abs(error) < PEDRO_DEADBAND_DEG) {
            turret.setPower(0);
            lastError = error;
            return;
        }

        double d = error - lastError;
        lastError = error;

        double power = PEDRO_kP * error + PEDRO_kD * d;
        power = clamp(power, -PEDRO_MAX_POWER, PEDRO_MAX_POWER);

        turret.setPower(power);
    }

    /* ================= LIMELIGHT TY TRACK (ABSOLUTE, LIMITED) ================= */
    private boolean limelightTrackTY() {

        LLResult ll = limelight.getLatestResult();
        if (ll == null || !ll.isValid()) {
            turret.setPower(0);
            return false;
        }

        double ty = ll.getTy() - LL_TY_OFFSET;

        if (Math.abs(ty) < LL_DEADBAND) {
            turret.setPower(0);
            return true;
        }

        int currentTicks = turret.getCurrentPosition();
        int correction = (int) (-ty * LL_TICKS_PER_TY);
        int targetTicks = currentTicks + correction;

        targetTicks = clamp(
                targetTicks,
                TURRET_LEFT_LIMIT,
                TURRET_RIGHT_LIMIT
        );

        int errorTicks = targetTicks - currentTicks;

        double power = clamp(
                errorTicks * LL_kP,
                -LL_MAX_POWER,
                LL_MAX_POWER
        );

        turret.setPower(power);
        return false;
    }

    /* ================= SHOOT ================= */
    private void alignAndShoot() {

        limelightOverride = true;

        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 900) {
            limelightTrackTY();
        }

        double rpm =
                Math.abs(shooterL.getVelocity()) / TICKS_PER_REV * 60.0;
        if (rpm < RPM_READY) {
            limelightOverride = false;
            return;
        }

        leftRamp.setPosition(0.55);
        rightRamp.setPosition(0.45);
        sleep(1300);
        leftRamp.setPosition(0.3);
        rightRamp.setPosition(0.7);

        limelightOverride = false;
    }

    /* ================= PATH ================= */
    private void runPath(PathChain path, double speed) {

        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            if (!limelightOverride) updateTurretPedro();
        }
    }

    /* ================= UTILS ================= */
    private void setShooterRPM(double rpm) {
        double ticks = rpm / 60.0 * TICKS_PER_REV;
        shooterL.setVelocity(ticks);
        shooterR.setVelocity(ticks);
    }

    private double normalize(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private int clamp(int v, int min, int max) {
        return Math.max(min, Math.min(max, v));
    }

    /* ================= PATHS ================= */
    public static class Paths {
        public PathChain Path1,Path2,Path3,Path4,Path5,
                Path6,Path7,Path8,Path9,Path10,Path11;

        public Paths(Follower f) {
            Path1  = line(f,19.664,123.514,46.355,96.785,315,180);
            Path2  = line(f,46.355,96.785,45,84.346,180,180);
            Path3  = line(f,45,84.346,22.907,84.224,180,180);
            Path4  = line(f,22.907,84.224,46.355,96.776,180,180);
            Path5  = line(f,46.355,96.776,45,60.215,180,180);
            Path6  = line(f,45,60.215,23.318,60.243,180,180);
            Path7  = line(f,23.318,60.243,46.561,96.757,180,180);
            Path8  = line(f,46.561,96.757,45,36.234,180,180);
            Path9  = line(f,45,36.234,23.318,36.019,180,180);
            Path10 = line(f,23.318,36.019,46,96,180,180);
            Path11 = line(f,46,96,37.234,71.776,180,90);
        }

        private static PathChain line(
                Follower f,
                double x1,double y1,double x2,double y2,
                double h1,double h2) {

            return f.pathBuilder()
                    .addPath(new BezierLine(new Pose(x1,y1),
                            new Pose(x2,y2)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(h1),
                            Math.toRadians(h2))
                    .build();
        }
    }
}
