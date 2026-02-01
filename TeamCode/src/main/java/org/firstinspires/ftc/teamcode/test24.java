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
@TeleOp(name = "test24")
public class test24 extends OpMode {

    // ================= CORE =================
    private Follower follower;
    private TelemetryManager telemetryM;
    private boolean automatedDrive = false;

    // ================= HARDWARE =================
    private DcMotor intake;
    private DcMotorEx shooterL, shooterR;
    private DcMotorEx turret;
    private Servo hood, leftRamp, rightRamp, leftLift, rightLift;
    private Limelight3A limelight;

    // ================= CONSTANTS =================
    static final double TICKS_PER_REV = 28.0;
    static final double SLOW_MODE = 0.5;
    static final double INTAKE_POWER = -0.72;
    static final double PRESET_INTAKE_POWER = -0.75;
    static final double RAMP_MIN_RPM = 2000;
    static final long FEED_TIME_MS = 1500;
    static final long POSE_WAIT_MS = 400;

    // ===== Shooter PIDF =====
    public static double SHOOTER_kP = 85;
    public static double SHOOTER_kI = 0.0;
    public static double SHOOTER_kD = 12;
    public static double SHOOTER_kF = 18;

    // ===== Turret constants (EXISTING) =====
    static final double TURRET_kP = 0.0028;
    static final double TURRET_kD = 0.05;
    static final double TURRET_DEADBAND = 0.15;
    static final double TURRET_MAX_POWER = 0.8;
    static final double TURRET_SLEW = 0.04;
    static final double TURRET_MANUAL_POWER = 0.65;
    static final double TURRET_ALIGN_DAMP = 0.6;
    static final double TURRET_TURN_COMP = 1.07;
    static final int TURRET_MIN_TICKS = -430;
    static final int TURRET_MAX_TICKS = 430;
    static final double TURN_SCALE = 0.6;

    // ===== Turret PD (FROM FINAL TUNER – ADDED) =====
    public static double TURRET_kF = 0.17;

    // ================= TURRET STATE =================
    private double lastTurretError = 0;
    private double lastTurretPower = 0;
    private double lastTurnInput = 0;

    // ===== PD TICK STATE (ADDED, NOT USED UNLESS CALLED) =====
    private int targetTicks = 0;
    private double lastError = 0;
    private double lastPower = 0;

    // ================= POSES =================
    static final Pose START_POSE = new Pose(37.234, 71.776, Math.toRadians(90));
    static final Pose CLOSE_SHOT_POSE = new Pose(46.355, 96.785, Math.toRadians(135));
    static final Pose LONG_SHOT_POSE  = new Pose(61.832, 12.935, Math.toRadians(105));
    static final Pose CLASSIFIER_APPROACH_POSE = new Pose(32.449, 70.991, Math.toRadians(0));
    static final Pose CLASSIFIER_OPEN_POSE     = new Pose(15.9, 70.776, Math.toRadians(0));

    // ================= PATH SPEEDS =================
    static final double SPEED_SHOOT = 0.9;
    static final double SPEED_CLASSIFIER_APPROACH = 0.9;
    static final double SPEED_CLASSIFIER_OPEN = 0.6;

    // ================= SERVOS =================
    static final double RAMP_UP_LEFT = 0.55;
    static final double RAMP_UP_RIGHT = 0.45;
    static final double RAMP_DOWN_LEFT = 0.3;
    static final double RAMP_DOWN_RIGHT = 0.7;

    static final double LIFT_UP_LEFT = 1.0;
    static final double LIFT_UP_RIGHT = 0.12;
    static final double LIFT_DOWN_LEFT = 0.2;
    static final double LIFT_DOWN_RIGHT = 0.8;

    static final double SHORT_RPM = 3000;
    static final double SHORT_HOOD = 0.34;
    static final double LONG_RPM = 4090;
    static final double LONG_HOOD = 0.28;

    static final double CLOSE_PRESET_RPM = 3000;
    static final double CLOSE_PRESET_HOOD = 0.35;
    static final double LONG_PRESET_RPM = 3800;
    static final double LONG_PRESET_HOOD = 0.28;

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

        double speedMul = gamepad1.right_bumper ? SLOW_MODE : 0.5;
        lastTurnInput = -gamepad1.right_stick_x * TURN_SCALE * speedMul;
        if (Math.abs(lastTurnInput) < 0.02) lastTurnInput = 0;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedMul,
                -gamepad1.left_stick_x * speedMul,
                lastTurnInput,
                true
        );

        // EXISTING LIMELIGHT ALIGN
        alignTurret(gamepad2.left_bumper);

        // OPTIONAL: CALL THIS WHEN YOU WANT TICK-BASED PD
        // updateTurretPID();
    }

    // ================= LIMELIGHT TURRET ALIGN (UNCHANGED) =================
    private void alignTurret(boolean enable) {

        double power = 0;

        LLResult ll = limelight.getLatestResult();
        boolean hasTarget = ll != null && ll.isValid();

        if (enable && hasTarget) {
            double error = ll.getTy();

            if (Math.abs(error) > TURRET_DEADBAND) {
                double derivative = error - lastTurretError;
                power += (TURRET_kP * error) + (TURRET_kD * derivative);
                power += Math.signum(error) * TURRET_kF;
                lastTurretError = error;
            } else {
                lastTurretError = 0;
            }
        }

        power += lastTurnInput * TURRET_TURN_COMP;
        power *= TURRET_ALIGN_DAMP;

        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        double delta = clamp(power - lastTurretPower, -TURRET_SLEW, TURRET_SLEW);
        power = lastTurretPower + delta;
        lastTurretPower = power;

        int pos = turret.getCurrentPosition();
        if ((power > 0 && pos >= TURRET_MAX_TICKS) ||
                (power < 0 && pos <= TURRET_MIN_TICKS)) {
            power = 0;
        }

        turret.setPower(power);
    }

    // ================= TURRET PD (FROM FINAL TUNER – FULL) =================
    private void updateTurretPID() {

        int currentTicks = turret.getCurrentPosition();
        double error = targetTicks - currentTicks;
        double derivative = error - lastError;

        double power = (TURRET_kP * error) + (TURRET_kD * derivative);

        if (Math.abs(error) > 5) {
            power += Math.signum(error) * TURRET_kF;
        }

        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        double delta = clamp(power - lastPower, -TURRET_SLEW, TURRET_SLEW);
        power = lastPower + delta;

        if ((power > 0 && currentTicks >= TURRET_MAX_TICKS) ||
                (power < 0 && currentTicks <= TURRET_MIN_TICKS)) {
            power = 0;
        }

        turret.setPower(power);

        lastError = error;
        lastPower = power;
    }

    // ================= UTILS =================
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);
    }
}
