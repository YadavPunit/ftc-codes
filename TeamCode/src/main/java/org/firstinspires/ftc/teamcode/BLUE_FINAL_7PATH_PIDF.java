package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE_FINAL_7PATH_PIDF", group = "Auto")
public class BLUE_FINAL_7PATH_PIDF extends LinearOpMode {

    /* ================= PEDRO ================= */
    private Follower follower;
    private Paths paths;

    /* ================= HARDWARE ================= */
    private DcMotorEx shooterL, shooterR, turret;
    private DcMotor intake;
    private Servo hood, leftRamp, rightRamp;
    private Limelight3A limelight;

    /* ================= PATH SPEED ================= */
    enum PathSpeed {
        FAST(0.90),
        MEDIUM(0.75),
        APPROACH(0.55),
        SHOOT(0.45);

        public final double power;
        PathSpeed(double p) { power = p; }
    }

    /* ================= SHOT PROFILE ================= */
    static class ShotProfile {
        final double rpm;          // shooter RPM
        final double feed;         // intake feed power
        final double tyOffset;     // limelight offset
        final long extraWaitMs;    // extra shoot wait if needed

        ShotProfile(double rpm, double feed, double tyOffset, long wait) {
            this.rpm = rpm;
            this.feed = feed;
            this.tyOffset = tyOffset;
            this.extraWaitMs = wait;
        }
    }

    /* ======= RPM LOGIC =======
       Short distance  → LOW RPM
       Medium distance → MID RPM
       Far distance    → HIGH RPM
    */

    static final ShotProfile SHORT_SHOT =
            new ShotProfile(2600, 0.45, 0.10, 200);

    static final ShotProfile MID_SHOT =
            new ShotProfile(3200, 0.50, 0.25, 250);

    static final ShotProfile FAR_SHOT =
            new ShotProfile(3800, 0.60, 0.35, 300);

    private ShotProfile currentShot;

    /* ================= SHOOTER PIDF ================= */
    static final double TICKS_PER_REV = 28.0;
    static final double SHOOTER_kP = 90;
    static final double SHOOTER_kI = 0;
    static final double SHOOTER_kD = 15;
    static final double SHOOTER_kF = 18;

    private double targetVelocity = 0;

    /* ================= TURRET PIDF (TY BASED) ================= */
    static final double TURRET_kP = 0.085;
    static final double TURRET_kI = 0.002;
    static final double TURRET_kD = 0.012;
    static final double TURRET_kF = 0.06;

    static final double TURRET_DEADBAND = 0.12;
    static final double TURRET_MAX_POWER = 0.8;
    static final double TURRET_SLEW = 0.035;
    static final double INTEGRAL_LIMIT = 0.35;

    static final int TURRET_MIN_TICKS = -430;
    static final int TURRET_MAX_TICKS = 430;

    private double turretIntegral = 0;
    private double lastTurretError = 0;
    private double lastTurretPower = 0;
    private double turretOffset = 0;

    /* ================= SHOOT CONTROL ================= */
    static final long BASE_SHOOT_TIME = 1100;
    static final double RPM_TOLERANCE = 50;
    static final double INTAKE_COLLECT = 0.65;

    private boolean shooting = false;
    private long shootStart = 0;

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

        shooterL.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterR.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hood.setPosition(0.25);
        rampDown();

        paths = new Paths(follower);

        waitForStart();
        if (isStopRequested()) return;

        intake.setPower(INTAKE_COLLECT);
        startPath(paths.Path1, PathSpeed.FAST, MID_SHOT);

        while (opModeIsActive() && state != AutoState.DONE) {

            follower.update();
            updateShooter();
            updateTurretPIDF();

            switch (state) {

                case PATH1:
                    if (!follower.isBusy()) state = AutoState.SHOOT1;
                    break;

                case SHOOT1:
                    handleShooting(() -> {
                        startPath(paths.Path2, PathSpeed.FAST, null);
                        state = AutoState.PATH2;
                    });
                    break;

                case PATH2:
                    if (!follower.isBusy()) {
                        startPath(paths.Path3, PathSpeed.APPROACH, MID_SHOT);
                        state = AutoState.PATH3;
                    }
                    break;

                case PATH3:
                    if (!follower.isBusy()) state = AutoState.SHOOT2;
                    break;

                case SHOOT2:
                    handleShooting(() -> {
                        startPath(paths.Path4, PathSpeed.MEDIUM, null);
                        state = AutoState.PATH4;
                    });
                    break;

                case PATH4:
                    if (!follower.isBusy()) {
                        startPath(paths.Path5, PathSpeed.APPROACH, FAR_SHOT);
                        state = AutoState.PATH5;
                    }
                    break;

                case PATH5:
                    if (!follower.isBusy()) state = AutoState.SHOOT3;
                    break;

                case SHOOT3:
                    handleShooting(() -> {
                        startPath(paths.Path6, PathSpeed.FAST, null);
                        state = AutoState.PATH6;
                    });
                    break;

                case PATH6:
                    if (!follower.isBusy()) {
                        startPath(paths.Path7, PathSpeed.APPROACH, FAR_SHOT);
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

    /* ================= PATH START ================= */
    private void startPath(PathChain path, PathSpeed speed, ShotProfile shot) {
        follower.setMaxPower(speed.power);
        follower.followPath(path);

        if (shot != null) {
            currentShot = shot;
            turretOffset = shot.tyOffset;
            setShooterRPM(shot.rpm);
        }
    }

    /* ================= SHOOT HANDLER ================= */
    private void handleShooting(Runnable onFinish) {

        if (!shooting && readyToShoot()) {
            shooting = true;
            shootStart = System.currentTimeMillis();
            rampUp();
            intake.setPower(currentShot.feed);
        }

        long shootDuration = BASE_SHOOT_TIME + currentShot.extraWaitMs;
        boolean timeout = System.currentTimeMillis() - shootStart > shootDuration;
        boolean rpmDrop = shooterL.getVelocity() < targetVelocity * 0.85;

        if (shooting && (timeout || rpmDrop)) {
            shooting = false;
            rampDown();
            intake.setPower(INTAKE_COLLECT);
            onFinish.run();
        }
    }

    /* ================= TURRET PIDF ================= */
    private void updateTurretPIDF() {

        LLResult ll = limelight.getLatestResult();
        if (ll == null || !ll.isValid()) {
            turret.setPower(lastTurretPower * 0.7);
            return;
        }

        double error = ll.getTy() + turretOffset;

        if (Math.abs(error) < TURRET_DEADBAND) {
            turretIntegral = 0;
            lastTurretError = error;
            turret.setPower(0);
            return;
        }

        turretIntegral += error;
        turretIntegral = clamp(turretIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        double derivative = error - lastTurretError;

        double power =
                TURRET_kP * error +
                        TURRET_kI * turretIntegral +
                        TURRET_kD * derivative +
                        TURRET_kF * Math.signum(error);

        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        double delta = clamp(power - lastTurretPower,
                -TURRET_SLEW, TURRET_SLEW);
        power = lastTurretPower + delta;

        lastTurretPower = power;
        lastTurretError = error;

        int pos = turret.getCurrentPosition();
        if ((power > 0 && pos >= TURRET_MAX_TICKS) ||
                (power < 0 && pos <= TURRET_MIN_TICKS)) {
            power = 0;
        }

        turret.setPower(power);
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
        return Math.abs(shooterL.getVelocity() - targetVelocity) < RPM_TOLERANCE;
    }

    private boolean readyToShoot() {
        return shooterAtSpeed() && Math.abs(lastTurretError) < 0.2;
    }

    /* ================= RAMPS ================= */
    private void rampUp() {
        leftRamp.setPosition(0.55);
        rightRamp.setPosition(0.45);
    }

    private void rampDown() {
        leftRamp.setPosition(0.30);
        rightRamp.setPosition(0.70);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /* ================= PATHS (ALL 7) ================= */
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower f) {

            Path1 = f.pathBuilder().addPath(
                    new BezierLine(new Pose(54.206, 7.327),
                            new Pose(59.140, 14.467))
            ).setLinearHeadingInterpolation(Math.toRadians(90),
                    Math.toRadians(180)).build();

            Path2 = f.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(59.140, 14.467),
                            new Pose(59.393, 37.668),
                            new Pose(61.682, 37.458),
                            new Pose(16.168, 36.925))
            ).setLinearHeadingInterpolation(
                    Math.toRadians(180), Math.toRadians(180)).build();

            Path3 = f.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(16.168, 36.925),
                            new Pose(59.178, 14.411))
            ).setLinearHeadingInterpolation(
                    Math.toRadians(180), Math.toRadians(180)).build();

            Path4 = f.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(59.178, 14.411),
                            new Pose(61.930, 65.972),
                            new Pose(54.509, 64.093),
                            new Pose(17.991, 62.374))
            ).setLinearHeadingInterpolation(
                    Math.toRadians(180), Math.toRadians(180)).build();

            Path5 = f.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(17.991, 62.374),
                            new Pose(50.836, 65.047),
                            new Pose(41.701, 95.308))
            ).setLinearHeadingInterpolation(
                    Math.toRadians(180), Math.toRadians(180)).build();

            Path6 = f.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(41.701, 95.308),
                            new Pose(43.206, 83.542),
                            new Pose(47.019, 83.374),
                            new Pose(18.935, 83.963))
            ).setLinearHeadingInterpolation(
                    Math.toRadians(180), Math.toRadians(180)).build();

            Path7 = f.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(18.935, 83.963),
                            new Pose(41.486, 95.411))
            ).setLinearHeadingInterpolation(
                    Math.toRadians(180), Math.toRadians(180)).build();
        }
    }
}