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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "test21_testmaping3")
public class test21_testmaping3 extends OpMode {

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
    static final double TURN_SCALE = 0.6;

    static final double INTAKE_POWER = -0.8;
    static final double PRESET_INTAKE_POWER = -0.75;

    static final double RAMP_MIN_RPM = 2000;
    static final long FEED_TIME_MS = 1500;
    static final long POSE_WAIT_MS = 400;

    /* ===== Shooter PIDF ===== */
    public static double SHOOTER_kP = 72.5;
    public static double SHOOTER_kI = 0.0;
    public static double SHOOTER_kD = 6;
    public static double SHOOTER_kF = 16.2;

    /* ===== Turret PIDF ===== */
    static final double TURRET_kP = 0.0028;
    static final double TURRET_kD = 0.05;
    static final double TURRET_kF = 0.17;

    static final double TY_OFFSET = 0.0;

    static final double TA_CLOSE = 1.6;
    static final double TA_FAR   = 0.4;

    static final double TY_DEADBAND_CLOSE = 0.04;
    static final double TY_DEADBAND_FAR   = 0.05;   // widened (IMPORTANT)

    static final double TURRET_MAX_POWER = 0.65;
    static final double TURRET_SLEW = 0.04;
    static final double LOSS_DECAY = 0.92;

    static final int TURRET_MIN_TICKS = -160;
    static final int TURRET_MAX_TICKS = 160;

    static final double D_ALPHA = 0.7;

    /* ================= TURRET STATE ================= */
    private double lastTurretError = 0;
    private double lastTurretPower = 0;
    private double filteredDerivative = 0;

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

    /* ================= SHOOTER ================= */
    static final double SHORT_RPM = 3000;
    static final double SHORT_HOOD = 0.34;
    static final double LONG_RPM = 4090;
    static final double LONG_HOOD = 0.28;

    /* ================= PRESETS ================= */
    static final double CLOSE_PRESET_RPM = 3000;
    static final double CLOSE_PRESET_HOOD = 0.35;
    static final double LONG_PRESET_RPM = 3800;
    static final double LONG_PRESET_HOOD = 0.28;

    /* ================= POSES ================= */
    static final Pose START_POSE = new Pose(37.234, 71.776, Math.toRadians(90));
    static final Pose CLOSE_SHOT_POSE = new Pose(46.355, 96.785, Math.toRadians(135));
    static final Pose LONG_SHOT_POSE  = new Pose(61.832, 12.935, Math.toRadians(105));
    static final Pose CLASSIFIER_APPROACH_POSE = new Pose(32.449, 70.991, Math.toRadians(0));
    static final Pose CLASSIFIER_OPEN_POSE     = new Pose(15.9, 70.776, Math.toRadians(0));

    private PathChain closeShotPath, longShotPath, classifierApproachPath, classifierOpenPath;

    /* ================= PRESET STATE ================= */
    private enum PresetType { NONE, CLOSE, LONG, CLASSIFIER }
    private enum PresetState { MOVE, WAIT_AT_POSE, WAIT_RPM, FEEDING, MOVE_2 }

    private PresetType activePreset = PresetType.NONE;
    private PresetState presetState;
    private long stateStartTime = 0;

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

        /* ===== SINGLE LIMELIGHT READ ===== */
        LLResult ll = limelight.getLatestResult();
        boolean hasTag = (ll != null && ll.isValid());

        /* ===== TURRET AUTO ALIGN (STABLE) ===== */
        if (gamepad2.left_bumper) {

            if (!hasTag) {
                lastTurretPower *= LOSS_DECAY;
                turret.setPower(lastTurretPower);
            } else {

                double error = ll.getTy() - TY_OFFSET;
                double ta = ll.getTa();

                double deadband = (ta > TA_CLOSE) ? TY_DEADBAND_CLOSE : TY_DEADBAND_FAR;

                if (Math.abs(error) < deadband) {
                    turret.setPower(0);
                    lastTurretPower = 0;
                    lastTurretError = 0;
                } else {

                    double rawDerivative = error - lastTurretError;
                    filteredDerivative = (D_ALPHA * filteredDerivative)
                            + ((1.0 - D_ALPHA) * rawDerivative);

                    // 🔑 DISTANCE-SCALED PID
                    double distanceScale = clamp(ta / TA_CLOSE, 0.25, 1.0);
                    double kP = TURRET_kP * distanceScale;
                    double kD = TURRET_kD * distanceScale;

                    // 🔇 DERIVATIVE DAMPING WHEN FAR
                    if (ta < TA_FAR) {
                        filteredDerivative *= 0.5;
                    }

                    double power = (kP * error) + (kD * filteredDerivative);

                    // 🔑 FEEDFORWARD ONLY WHEN ERROR IS LARGE
                    if (ta < TA_FAR && Math.abs(error) > 0.6) {
                        power += Math.signum(error) * TURRET_kF;
                    }

                    power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

                    double delta = clamp(power - lastTurretPower, -TURRET_SLEW, TURRET_SLEW);
                    power = lastTurretPower + delta;

                    int pos = turret.getCurrentPosition();
                    if ((power > 0 && pos >= TURRET_MAX_TICKS) ||
                            (power < 0 && pos <= TURRET_MIN_TICKS)) {
                        power = 0;
                    }

                    turret.setPower(power);
                    lastTurretPower = power;
                    lastTurretError = error;
                }
            }
        } else {
            turret.setPower(0);
            lastTurretPower = 0;
            lastTurretError = 0;
        }

        /* ===== PRESETS ===== */
        if (gamepad1.triangle) startPreset(PresetType.CLOSE);
        else if (gamepad1.cross) startPreset(PresetType.LONG);
        else if (gamepad1.square) startPreset(PresetType.CLASSIFIER);
        else if (gamepad1.circle) stopPreset();

        if (automatedDrive) runPreset();

        /* ===== DRIVE ===== */
        double speedMul = gamepad1.right_bumper ? SLOW_MODE : 0.5;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedMul,
                -gamepad1.left_stick_x * speedMul,
                -gamepad1.right_stick_x * TURN_SCALE * speedMul,
                true
        );

        /* ===== LIFT ===== */
        if (gamepad1.dpad_up) setLift(true);
        else if (gamepad1.dpad_down) setLift(false);

        /* ===== INTAKE ===== */
        double intakePower = 0.0;
        boolean triggerActive = false;

        if (gamepad1.left_trigger > 0.05) {
            intakePower = -0.8 * gamepad1.left_trigger;
            triggerActive = true;
        } else if (gamepad1.right_trigger > 0.05) {
            intakePower = 1.0 * gamepad1.right_trigger;
            triggerActive = true;
        }

        if (triggerActive) {
            intake.setPower(intakePower);
        } else {
            if (gamepad2.cross && !prevX) intakeToggle = !intakeToggle;
            prevX = gamepad2.cross;
            intake.setPower(intakeToggle ? INTAKE_POWER : 0);
        }

        /* ===== SHOOTER ===== */
        if (gamepad2.left_bumper && hasTag) {
            double tx = ll.getTx();
            double rpm = map(tx, 1.14, 15.77, 2540, 3600);
            runShooterRPM(rpm);
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
        if (rpmNow < RAMP_MIN_RPM) {
            setRamp(false);
        } else {
            if (gamepad2.dpad_up) setRamp(true);
            else if (gamepad2.dpad_down) setRamp(false);
        }
    }

    /* ================= HELPERS ================= */
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

    private void stopPreset() {
        automatedDrive = false;
        activePreset = PresetType.NONE;

        follower.startTeleopDrive();
        follower.setMaxPower(1.0);

        shooterL.setVelocity(0);
        shooterR.setVelocity(0);
        intake.setPower(0);
        setRamp(false);
    }

    private void runPreset() {

        boolean pathDone = !follower.isBusy();
        double rpm = Math.abs(shooterL.getVelocity()) / TICKS_PER_REV * 60.0;

        switch (presetState) {

            case MOVE:
                if (pathDone) {
                    presetState = PresetState.WAIT_AT_POSE;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case WAIT_AT_POSE:
                if (System.currentTimeMillis() - stateStartTime >= POSE_WAIT_MS) {

                    if (activePreset == PresetType.CLASSIFIER) {
                        follower.setMaxPower(SPEED_CLASSIFIER_OPEN);
                        follower.followPath(classifierOpenPath);
                        presetState = PresetState.MOVE_2;
                    } else {
                        intake.setPower(PRESET_INTAKE_POWER);
                        runShooterRPM(activePreset == PresetType.CLOSE ? CLOSE_PRESET_RPM : LONG_PRESET_RPM);
                        hood.setPosition(activePreset == PresetType.CLOSE ? CLOSE_PRESET_HOOD : LONG_PRESET_HOOD);
                        presetState = PresetState.WAIT_RPM;
                    }
                }
                break;

            case MOVE_2:
                if (pathDone) stopPreset();
                break;

            case WAIT_RPM:
                if (rpm >= RAMP_MIN_RPM) {
                    setRamp(true);
                    stateStartTime = System.currentTimeMillis();
                    presetState = PresetState.FEEDING;
                }
                break;

            case FEEDING:
                if (System.currentTimeMillis() - stateStartTime >= FEED_TIME_MS) {
                    stopPreset();
                }
                break;
        }
    }

    private void startPreset(PresetType type) {
        if (automatedDrive) return;

        automatedDrive = true;
        activePreset = type;
        presetState = PresetState.MOVE;
        stateStartTime = System.currentTimeMillis();

        intake.setPower(0);
        setRamp(false);

        if (type == PresetType.CLOSE) {
            follower.setMaxPower(SPEED_SHOOT);
            follower.followPath(closeShotPath);
        } else if (type == PresetType.LONG) {
            follower.setMaxPower(SPEED_SHOOT);
            follower.followPath(longShotPath);
        } else if (type == PresetType.CLASSIFIER) {
            follower.setMaxPower(SPEED_CLASSIFIER_APPROACH);
            follower.followPath(classifierApproachPath);
        }
    }

    private double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);
    }
}
