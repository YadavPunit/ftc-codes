package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "test16_with_preset_STABLE_FINAL")
public class test16_with_preset_FINAL extends OpMode {

    // ================= CORE =================
    private Follower follower;
    private TelemetryManager telemetryM;
    private boolean automatedDrive = false;

    // ================= HARDWARE =================
    private DcMotor intake;
    private DcMotorEx shooterL, shooterR;
    private Servo hood, leftRamp, rightRamp, leftLift, rightLift;

    // ================= CONSTANTS =================
    static final double TICKS_PER_REV = 28.0;
    static final double INTAKE_POWER = -0.8;
    static final double PRESET_INTAKE_POWER = -0.75;
    static final double SLOW_MODE = 0.5;
    static final double RAMP_MIN_RPM = 2500;
    static final long FEED_TIME_MS = 1500;
    static final long POSE_WAIT_MS = 1000;

    // ===== Path speeds =====
    static final double SPEED_SHOOT = 0.5;
    static final double SPEED_CLASSIFIER_APPROACH = 0.45;
    static final double SPEED_CLASSIFIER_OPEN = 0.35;

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

    // ===== Poses =====
    static final Pose START_POSE = new Pose(37.234, 71.776, Math.toRadians(90));
    static final Pose CLOSE_SHOT_POSE = new Pose(46.355, 96.785, Math.toRadians(133));
    static final Pose LONG_SHOT_POSE  = new Pose(61.832, 12.935, Math.toRadians(105));
    static final Pose CLASSIFIER_APPROACH_POSE = new Pose(32.449, 70.991, Math.toRadians(0));
    static final Pose CLASSIFIER_OPEN_POSE     = new Pose(18.336, 70.776, Math.toRadians(0));

    // ================= PATHS =================
    private PathChain closeShotPath, longShotPath, classifierApproachPath, classifierOpenPath;

    // ================= PRESET STATE =================
    private enum PresetType { NONE, CLOSE, LONG, CLASSIFIER }
    private enum PresetState { MOVE, WAIT_AT_POSE, WAIT_RPM, FEEDING, MOVE_2 }

    private PresetType activePreset = PresetType.NONE;
    private PresetState presetState;
    private long stateStartTime = 0;

    // ================= MANUAL STATE =================
    private boolean intakeToggle = false;
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

        hood = hardwareMap.get(Servo.class, "hood");
        leftRamp = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");
        leftLift = hardwareMap.get(Servo.class, "left_lift");
        rightLift = hardwareMap.get(Servo.class, "right_lift");

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

        // ===== PRESETS (GP1) =====
        if (gamepad1.triangle) startPreset(PresetType.CLOSE);
        else if (gamepad1.cross) startPreset(PresetType.LONG);
        else if (gamepad1.square) startPreset(PresetType.CLASSIFIER);
        else if (gamepad1.circle) stopPreset();

        if (automatedDrive) {
            runPreset();
            return;
        }

        // ================= MANUAL TELEOP =================

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

        // INTAKE TOGGLE (GP2)
        if (gamepad2.cross && !prevX) intakeToggle = !intakeToggle;
        prevX = gamepad2.cross;
        intake.setPower(intakeToggle ? INTAKE_POWER : 0);

        // SHOOTER (GP2)
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

        // RAMP SAFETY
        double rpm = Math.abs(shooterL.getVelocity()) / TICKS_PER_REV * 60.0;
        if (rpm < RAMP_MIN_RPM) {
            setRamp(false);
        } else {
            if (gamepad2.dpad_up) setRamp(true);
            else if (gamepad2.dpad_down) setRamp(false);
        }
    }

    // ================= PRESET LOGIC =================
    private void startPreset(PresetType type) {
        if (automatedDrive) return;

        automatedDrive = true;
        activePreset = type;
        presetState = PresetState.MOVE;
        stateStartTime = System.currentTimeMillis();

        setRamp(false);
        intake.setPower(0);

        if (type == PresetType.CLOSE) {
            follower.setMaxPower(SPEED_SHOOT);
            follower.followPath(closeShotPath);
        } else if (type == PresetType.LONG) {
            follower.setMaxPower(SPEED_SHOOT);
            follower.followPath(longShotPath);
        } else {
            follower.setMaxPower(SPEED_CLASSIFIER_APPROACH);
            follower.followPath(classifierApproachPath);
        }
    }

    private void stopPreset() {
        activePreset = PresetType.NONE;
        automatedDrive = false;
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
                if (System.currentTimeMillis() - stateStartTime >= FEED_TIME_MS)
                    stopPreset();
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
