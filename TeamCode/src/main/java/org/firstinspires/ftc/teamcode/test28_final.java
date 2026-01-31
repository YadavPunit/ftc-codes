package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "test28_final")
public class test28_final extends OpMode {

    /* ================= CORE ================= */
    private Follower follower;
    private TelemetryManager telemetryM;
    private boolean automatedDrive = false;

    /* ================= HARDWARE ================= */
    private DcMotor intake;
    private DcMotorEx shooterL, shooterR;
    private DcMotorEx turret;
    private Servo hood, leftRamp, rightRamp, leftLift, rightLift;
    private Limelight3A limelight;

    /* ================= CONSTANTS ================= */

    static final double TICKS_PER_REV = 28.0;
    static final double SLOW_MODE = 0.5;
    static final double INTAKE_POWER = -0.8;
    static final double PRESET_INTAKE_POWER = -0.75;
    static final double RAMP_MIN_RPM = 2300;
    static final long FEED_TIME_MS = 1500;
    static final long POSE_WAIT_MS = 400;

    /* ===== Shooter PIDF (from test21) ===== */
    public static double SHOOTER_kP = 72.5;
    public static double SHOOTER_kI = 0.0;
    public static double SHOOTER_kD = 6;
    public static double SHOOTER_kF = 16.2;

    /* ===== Turret Pedro + Limelight MICRO (test28) ===== */
    public static double TURRET_kP = 0.019;
    public static double TURRET_kD = 0.1;

    public static double LL_MICRO_kP = 0.015;
    public static double LL_MICRO_MAX = 0.15;
    public static double LL_MICRO_DEADBAND = 0.4;

    public static double TURRET_MAX_POWER = 0.65;
    public static double TURRET_SLEW = 0.04;
    public static double TURRET_ALIGN_DAMP = 0.6;
    public static double TURRET_TURN_COMP = 0;

    static final double TICKS_PER_DEGREE = (28.0 * 18.0) / 360.0;
    static final int TURRET_MIN_TICKS = -160;
    static final int TURRET_MAX_TICKS = 160;

    static final double TURN_SCALE = 0.6;

    /* ================= STATE ================= */
    private double lastTurretError = 0;
    private double lastTurretPower = 0;
    private double lastTurnInput = 0;

    /* ================= POSES ================= */
    static final Pose START_POSE = new Pose(37.234, 71.776, Math.toRadians(90));
    static final Pose CLOSE_SHOT_POSE = new Pose(46.355, 96.785, Math.toRadians(135));
    static final Pose LONG_SHOT_POSE  = new Pose(61.832, 12.935, Math.toRadians(105));
    static final Pose CLASSIFIER_APPROACH_POSE = new Pose(32.449, 70.991, Math.toRadians(0));
    static final Pose CLASSIFIER_OPEN_POSE     = new Pose(15.9, 70.776, Math.toRadians(0));

    /* ================= PATH SPEEDS ================= */
    static final double SPEED_SHOOT = 0.9;
    static final double SPEED_CLASSIFIER_APPROACH = 0.9;
    static final double SPEED_CLASSIFIER_OPEN = 0.6;

    /* ================= RAMP ================= */
    static final double RAMP_UP_LEFT = 0.55;
    static final double RAMP_UP_RIGHT = 0.45;
    static final double RAMP_DOWN_LEFT = 0.3;
    static final double RAMP_DOWN_RIGHT = 0.7;

    /* ================= LIFT ================= */
    static final double LIFT_UP_LEFT = 1.0;
    static final double LIFT_UP_RIGHT = 0.12;
    static final double LIFT_DOWN_LEFT = 0.2;
    static final double LIFT_DOWN_RIGHT = 0.8;

    /* ================= SHOOTER PRESETS ================= */
    static final double SHORT_RPM = 3000;
    static final double SHORT_HOOD = 0.34;
    static final double LONG_RPM = 4090;
    static final double LONG_HOOD = 0.28;

    static final double CLOSE_PRESET_RPM = 3000;
    static final double CLOSE_PRESET_HOOD = 0.35;
    static final double LONG_PRESET_RPM = 3800;
    static final double LONG_PRESET_HOOD = 0.28;

    /* ================= PATHS ================= */
    private PathChain closeShotPath, longShotPath, classifierApproachPath, classifierOpenPath;

    /* ================= PRESET STATE ================= */
    private enum PresetType { NONE, CLOSE, LONG, CLASSIFIER }
    private enum PresetState { MOVE, WAIT_AT_POSE, WAIT_RPM, FEEDING, MOVE_2 }

    private PresetType activePreset = PresetType.NONE;
    private PresetState presetState;
    private long stateStartTime = 0;

    /* ================= MANUAL ================= */
    private boolean intakeToggle = false;
    private boolean prevX = false;

    /* ================= INIT ================= */
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterL = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterR = hardwareMap.get(DcMotorEx.class, "right_shooting");
        shooterR.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterR.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        leftRamp = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");
        leftLift = hardwareMap.get(Servo.class, "left_lift");
        rightLift = hardwareMap.get(Servo.class, "right_lift");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        closeShotPath = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, CLOSE_SHOT_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), CLOSE_SHOT_POSE.getHeading())
                .build();

        longShotPath = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, LONG_SHOT_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), LONG_SHOT_POSE.getHeading())
                .build();

        classifierApproachPath = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, CLASSIFIER_APPROACH_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), CLASSIFIER_APPROACH_POSE.getHeading())
                .build();

        classifierOpenPath = follower.pathBuilder()
                .addPath(new BezierLine(CLASSIFIER_APPROACH_POSE, CLASSIFIER_OPEN_POSE))
                .setLinearHeadingInterpolation(CLASSIFIER_APPROACH_POSE.getHeading(), CLASSIFIER_OPEN_POSE.getHeading())
                .build();

        setRamp(false);
        setLift(false);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /* ================= LOOP ================= */
    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        if (gamepad1.triangle) startPreset(PresetType.CLOSE);
        else if (gamepad1.cross) startPreset(PresetType.LONG);
        else if (gamepad1.square) startPreset(PresetType.CLASSIFIER);
        else if (gamepad1.circle) stopPreset();

        if (automatedDrive) {
            runPreset();
            return;
        }

        /* ===== DRIVE ===== */
        double speedMul = gamepad1.right_bumper ? SLOW_MODE : 0.5;
        lastTurnInput = -gamepad1.right_stick_x * TURN_SCALE * speedMul;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedMul,
                -gamepad1.left_stick_x * speedMul,
                lastTurnInput,
                true
        );

        /* ===== TURRET AUTO (PEDRO + LL MICRO) ===== */
        alignTurret();

        /* ===== LIFT ===== */
        if (gamepad1.dpad_up) setLift(true);
        else if (gamepad1.dpad_down) setLift(false);

        /* ===== INTAKE ===== */
        double intakePower = 0;
        boolean triggerActive = false;

        if (gamepad1.left_trigger > 0.05) {
            intakePower = -0.8 * gamepad1.left_trigger;
            triggerActive = true;
        } else if (gamepad1.right_trigger > 0.05) {
            intakePower = gamepad1.right_trigger;
            triggerActive = true;
        }

        if (triggerActive) intake.setPower(intakePower);
        else {
            if (gamepad2.cross && !prevX) intakeToggle = !intakeToggle;
            prevX = gamepad2.cross;
            intake.setPower(intakeToggle ? INTAKE_POWER : 0);
        }

        /* ===== SHOOTER TX → RPM (FROM test21) ===== */
        if (gamepad2.left_bumper) {

            LLResult ll = limelight.getLatestResult();
            if (ll != null && ll.isValid()) {

                double tx = ll.getTx();
                double rpm, hoodPos;

                if (tx >= 15.77) {
                    rpm = 3820;
                    hoodPos = 0.25;
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
                    rpm = 2470;
                    hoodPos = 0.89;
                }

                runShooterRPM(rpm);
                hood.setPosition(hoodPos);
            }
        } else {
            if (gamepad2.triangle) {
                runShooterRPM(SHORT_RPM);
                hood.setPosition(SHORT_HOOD);
            } else if (gamepad2.circle) {
                runShooterRPM(LONG_RPM);
                hood.setPosition(LONG_HOOD);
            } else if (gamepad2.square) {
                shooterL.setVelocity(0);
                shooterR.setVelocity(0);
            }
        }

        /* ===== RAMP SAFETY ===== */
        double rpmNow = Math.abs(shooterL.getVelocity()) / TICKS_PER_REV * 60.0;
        if (rpmNow < RAMP_MIN_RPM) setRamp(false);
        else {
            if (gamepad2.dpad_up) setRamp(true);
            else if (gamepad2.dpad_down) setRamp(false);
        }
    }

    /* ================= TURRET ALIGN ================= */
    private void alignTurret() {

        Pose pose = follower.getPose();

        double fieldAngleDeg = Math.toDegrees(
                Math.atan2(131 - pose.getY(), 31 - pose.getX())
        );

        double targetDeg = normalize(fieldAngleDeg - Math.toDegrees(pose.getHeading()));
        double currentDeg = turret.getCurrentPosition() / TICKS_PER_DEGREE;
        double error = targetDeg - currentDeg;

        double derivative = error - lastTurretError;
        lastTurretError = error;

        double power = (TURRET_kP * error) + (TURRET_kD * derivative);

        LLResult ll = limelight.getLatestResult();
        if (ll != null && ll.isValid()) {
            double tx = ll.getTx();
            if (Math.abs(tx) > LL_MICRO_DEADBAND) {
                power += clamp(tx * LL_MICRO_kP, -LL_MICRO_MAX, LL_MICRO_MAX);
            }
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

    /* ================= PRESET LOGIC ================= */
    private void startPreset(PresetType type) {
        if (automatedDrive) return;
        automatedDrive = true;
        activePreset = type;
        presetState = PresetState.MOVE;
        stateStartTime = System.currentTimeMillis();

        if (type == PresetType.CLOSE) follower.followPath(closeShotPath);
        else if (type == PresetType.LONG) follower.followPath(longShotPath);
        else follower.followPath(classifierApproachPath);
    }

    private void stopPreset() {
        automatedDrive = false;
        follower.startTeleopDrive();
        shooterL.setVelocity(0);
        shooterR.setVelocity(0);
        intake.setPower(0);
        setRamp(false);
    }

    private void runPreset() {
        if (!follower.isBusy()) stopPreset();
    }

    /* ================= HELPERS ================= */
    private double normalize(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private void runShooterRPM(double rpm) {
        double ticks = (rpm / 60.0) * TICKS_PER_REV;
        shooterL.setVelocity(ticks);
        shooterR.setVelocity(ticks);
    }

    private void setRamp(boolean up) {
        leftRamp.setPosition(up ? RAMP_UP_LEFT : RAMP_DOWN_LEFT);
        rightRamp.setPosition(up ? RAMP_UP_RIGHT : RAMP_DOWN_RIGHT);
    }

    private void setLift(boolean up) {
        leftLift.setPosition(up ? LIFT_UP_LEFT : LIFT_DOWN_LEFT);
        rightLift.setPosition(up ? LIFT_UP_RIGHT : LIFT_DOWN_RIGHT);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);
    }
}
