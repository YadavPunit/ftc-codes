package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "test16_with_preset")
public class test16_with_preset extends OpMode {

    // ================= CORE =================
    private Follower follower;
    private TelemetryManager telemetryM;

    // ================= HARDWARE =================
    private DcMotor intake;
    private DcMotorEx shooterL, shooterR, turret;
    private Servo hood, leftRamp, rightRamp, leftLift, rightLift;
    private Limelight3A limelight;

    // ================= CONSTANTS =================
    static final double TICKS_PER_REV = 28.0;
    static final double INTAKE_POWER = -0.8;
    static final double PRESET_INTAKE_POWER = -0.75;
    static final double SLOW_MODE = 0.5;
    static final double RAMP_MIN_RPM = 2500;   // 🔐 SAFETY THRESHOLD
    static final long FEED_TIME_MS = 1500;

    // ===== Ramp =====
    static final double RAMP_UP_LEFT = 0.55;
    static final double RAMP_UP_RIGHT = 0.45;
    static final double RAMP_DOWN_LEFT = 0.3;
    static final double RAMP_DOWN_RIGHT = 0.7;

    // ===== Lift =====
    static final double LIFT_UP_LEFT = 1.0;
    static final double LIFT_UP_RIGHT = 0.12;
    static final double LIFT_DOWN_LEFT = 0.2;
    static final double LIFT_DOWN_RIGHT = 0.8;

    // ===== Shooter =====
    static final double SHORT_RPM = 3040;
    static final double SHORT_HOOD = 0.34;
    static final double LONG_RPM = 3790;
    static final double LONG_HOOD = 0.28;

    static final double CLOSE_PRESET_RPM = 3000;
    static final double CLOSE_PRESET_HOOD = 0.35;
    static final double LONG_PRESET_RPM = 3800;
    static final double LONG_PRESET_HOOD = 0.28;

    // ===== Preset Poses =====
    static final Pose START_POSE = new Pose(37.234, 71.776, 90);
    static final Pose CLOSE_SHOT_POSE = new Pose(46.355, 96.785, Math.toRadians(133));
    static final Pose LONG_SHOT_POSE  = new Pose(61.832, 12.935, Math.toRadians(105));
    static final Pose CLASSIFIER_APPROACH_POSE = new Pose(32.449, 70.991, Math.toRadians(0));
    static final Pose CLASSIFIER_OPEN_POSE     = new Pose(18.336, 70.776, Math.toRadians(0));

    // ================= PATH CHAINS =================
    private PathChain closeShotPath;
    private PathChain longShotPath;
    private PathChain classifierApproachPath;
    private PathChain classifierOpenPath;

    // ================= PRESET STATE =================
    private enum PresetType { NONE, CLOSE, LONG, CLASSIFIER }
    private enum PresetState { IDLE, MOVE_1, MOVE_2, WAIT_RPM, FEEDING }

    private PresetType activePreset = PresetType.NONE;
    private PresetState presetState = PresetState.IDLE;
    private long feedStartTime = 0;

    // ================= MANUAL STATE =================
    private boolean intakeToggle = false;
    private boolean shortShot = false;
    private boolean longShot = false;
    private boolean prevX = false;

    // ================= INIT =================
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterL = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterR = hardwareMap.get(DcMotorEx.class, "right_shooting");
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        hood = hardwareMap.get(Servo.class, "hood");
        leftRamp = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");
        leftLift = hardwareMap.get(Servo.class, "left_lift");
        rightLift = hardwareMap.get(Servo.class, "right_lift");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        // ===== BUILD PATH CHAINS =====
        closeShotPath = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, CLOSE_SHOT_POSE))
                .build();

        longShotPath = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, LONG_SHOT_POSE))
                .build();

        classifierApproachPath = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, CLASSIFIER_APPROACH_POSE))
                .build();

        classifierOpenPath = follower.pathBuilder()
                .addPath(new BezierLine(CLASSIFIER_APPROACH_POSE, CLASSIFIER_OPEN_POSE))
                .build();

        // 🔐 RAMP SAFE DEFAULT
        setRamp(false);
        setLift(false);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        // ===== PRESET BUTTONS (GP1) =====
        if (gamepad1.triangle) startPreset(PresetType.CLOSE);
        else if (gamepad1.cross) startPreset(PresetType.LONG);
        else if (gamepad1.square) startPreset(PresetType.CLASSIFIER);
        else if (gamepad1.circle) stopPreset();

        // ===== PRESET ACTIVE =====
        if (activePreset != PresetType.NONE) {
            runPreset();
            return;
        }

        // ================= NORMAL TELEOP =================

        // DRIVE
        double speedMul = gamepad1.right_bumper ? SLOW_MODE : 1.0;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedMul,
                -gamepad1.left_stick_x * speedMul,
                -gamepad1.right_stick_x * speedMul,
                true
        );

        // LIFT
        if (gamepad1.dpad_up) setLift(true);
        else if (gamepad1.dpad_down) setLift(false);

        // INTAKE (GP2)
        if (gamepad2.cross && !prevX) intakeToggle = !intakeToggle;
        prevX = gamepad2.cross;
        intake.setPower(intakeToggle ? INTAKE_POWER : 0);

        // SHOOTER (GP2)
        if (gamepad2.triangle) {
            shortShot = true;
            longShot = false;
        } else if (gamepad2.circle) {
            longShot = true;
            shortShot = false;
        } else if (gamepad2.square) {
            shortShot = false;
            longShot = false;
        }

        if (shortShot) {
            runShooterRPM(SHORT_RPM);
            hood.setPosition(SHORT_HOOD);
        } else if (longShot) {
            runShooterRPM(LONG_RPM);
            hood.setPosition(LONG_HOOD);
        } else {
            shooterL.setVelocity(0);
            shooterR.setVelocity(0);
        }

        // ===== 🔐 RAMP SAFETY (MANUAL) =====
        double rpm = (Math.abs(shooterL.getVelocity()) / TICKS_PER_REV) * 60.0;

        if (rpm < RAMP_MIN_RPM) {
            // Disable D-pad & force ramp down
            setRamp(false);
        } else {
            // Allow D-pad control
            if (gamepad2.dpad_up) setRamp(true);
            else if (gamepad2.dpad_down) setRamp(false);
        }
    }

    // ================= PRESET LOGIC =================
    private void startPreset(PresetType type) {
        if (activePreset != PresetType.NONE) return;

        activePreset = type;
        presetState = PresetState.MOVE_1;
        feedStartTime = 0;

        setRamp(false);
        intake.setPower(0);

        if (type == PresetType.CLOSE) follower.followPath(closeShotPath);
        else if (type == PresetType.LONG) follower.followPath(longShotPath);
        else follower.followPath(classifierApproachPath);
    }

    private void stopPreset() {
        activePreset = PresetType.NONE;
        presetState = PresetState.IDLE;
        shooterL.setVelocity(0);
        shooterR.setVelocity(0);
        intake.setPower(0);
        setRamp(false);
    }

    private void runPreset() {

        boolean pathDone = !follower.isBusy();
        double rpm = (Math.abs(shooterL.getVelocity()) / TICKS_PER_REV) * 60.0;

        switch (presetState) {

            case MOVE_1:
                if (activePreset == PresetType.CLOSE || activePreset == PresetType.LONG) {
                    intake.setPower(PRESET_INTAKE_POWER);
                    runShooterRPM(activePreset == PresetType.CLOSE ? CLOSE_PRESET_RPM : LONG_PRESET_RPM);
                    hood.setPosition(activePreset == PresetType.CLOSE ? CLOSE_PRESET_HOOD : LONG_PRESET_HOOD);
                    if (pathDone) presetState = PresetState.WAIT_RPM;
                } else {
                    if (pathDone) {
                        follower.followPath(classifierOpenPath);
                        presetState = PresetState.MOVE_2;
                    }
                }
                break;

            case MOVE_2:
                if (pathDone) stopPreset();
                break;

            case WAIT_RPM:
                if (rpm >= RAMP_MIN_RPM) {
                    setRamp(true);
                    feedStartTime = System.currentTimeMillis();
                    presetState = PresetState.FEEDING;
                }
                break;

            case FEEDING:
                if (System.currentTimeMillis() - feedStartTime >= FEED_TIME_MS) {
                    stopPreset();
                }
                break;
        }
    }

    // ================= HELPERS =================
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
}
