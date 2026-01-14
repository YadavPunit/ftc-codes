package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "test15")
public class test15 extends OpMode {

    // ================= CORE =================
    private Follower follower;
    private TelemetryManager telemetryM;

    // ================= MOTORS =================
    private DcMotor intake;
    private DcMotorEx shooterL, shooterR;
    private DcMotorEx turret;

    // ================= SERVOS =================
    private Servo hood;
    private Servo leftRamp, rightRamp;
    private Servo leftLift, rightLift;

    // ================= LIMELIGHT =================
    private Limelight3A limelight;

    // ================= CONSTANTS =================
    static final double TICKS_PER_REV = 28.0;

    // Shooter PIDF
    static final double SHOOTER_kP = 60.0;
    static final double SHOOTER_kI = 0.0;
    static final double SHOOTER_kD = 6.0;
    static final double SHOOTER_kF = 12.0;

    static final double INTAKE_POWER = -0.8;
    static final double SLOW_MODE = 0.5;

    // ================= RAMP VALUES =================
    static final double RAMP_DOWN_LEFT = 0.3;
    static final double RAMP_DOWN_RIGHT = 0.7;
    static final double RAMP_UP_LEFT = 0.55;
    static final double RAMP_UP_RIGHT = 0.45;

    // ================= LIFT VALUES =================
    static final double LIFT_UP_LEFT = 1.0;
    static final double LIFT_UP_RIGHT = 0.12;
    static final double LIFT_DOWN_LEFT = 0.2;
    static final double LIFT_DOWN_RIGHT = 0.8;

    // ================= SHOOT PRESETS =================
    static final double SHORT_RPM = 3040;
    static final double SHORT_HOOD = 0.34;

    static final double LONG_RPM = 3790;
    static final double LONG_HOOD = 0.28;

    // ================= TURRET LIMITS =================
    static final int TURRET_MIN_TICKS = -189;
    static final int TURRET_MAX_TICKS = 189;

    // ================= TURRET PD =================
    static final double TURRET_kP = 0.0197;
    static final double TURRET_kD = 0.001;
    static final double TURRET_DEADBAND = 0.18;
    static final double TURRET_MAX_POWER = 0.35;
    static final double TURRET_SLEW = 0.04;
    static final double ERROR_ALPHA = 0.20;
    static final double TY_OFFSET = 0.0;

    // ================= STATE =================
    private boolean intakeToggle = false;
    private boolean shortShot = false;
    private boolean longShot = false;

    private boolean prevX = false;

    private double filteredError = 0;
    private double lastError = 0;
    private double lastTurretPower = 0;

    // ================= INIT =================
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterL = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterR = hardwareMap.get(DcMotorEx.class, "right_shooting");
        shooterR.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterR.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(0.2, 0.8);
        hood.setPosition(0.5);


        leftRamp = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");
        leftLift = hardwareMap.get(Servo.class, "left_lift");
        rightLift = hardwareMap.get(Servo.class, "right_lift");

        setRamp(false);
        setLift(false);

        leftRamp.setPosition(0.3);
        rightRamp.setPosition(0.7);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
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

        // ===== DRIVE =====
        boolean slowMode = gamepad1.right_bumper;
        double speedMul = slowMode ? SLOW_MODE : 1.0;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedMul,
                -gamepad1.left_stick_x * speedMul,
                -gamepad1.right_stick_x * speedMul,
                true
        );

        // ===== LIFT (GAMEPAD 1) =====
        if (gamepad1.dpad_up) setLift(true);
        else if (gamepad1.dpad_down) setLift(false);

        // ===== TURRET ALIGN (LEFT BUMPER) =====
        alignTurret(gamepad1.left_bumper);

        // ===== INTAKE TOGGLE (GAMEPAD 2) =====
        if (gamepad2.cross && !prevX) {
            intakeToggle = !intakeToggle;
        }
        prevX = gamepad2.cross;
        intake.setPower(intakeToggle ? INTAKE_POWER : 0);

        // ===== RAMP CONTROL (GAMEPAD 2 DPAD) =====
        if (gamepad2.dpad_up) {
            setRamp(true);
        } else if (gamepad2.dpad_down) {
            setRamp(false);
        }

        // ===== SHOOTER CONTROL (GAMEPAD 2) =====
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

        // ===== SHOOTER OUTPUT =====
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



    }

    // ================= TURRET LOGIC =================
    private void alignTurret(boolean align) {

        LLResult ll = limelight.getLatestResult();
        boolean hasTarget = ll != null && ll.isValid();

        if (align && hasTarget) {

            double rawError = ll.getTy() - TY_OFFSET;
            filteredError = ERROR_ALPHA * rawError + (1 - ERROR_ALPHA) * filteredError;

            if (Math.abs(filteredError) < TURRET_DEADBAND) {
                stopTurret();
                return;
            }

            double derivative = filteredError - lastError;
            lastError = filteredError;

            double power = (TURRET_kP * filteredError) + (TURRET_kD * derivative);
            power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

            double delta = clamp(power - lastTurretPower, -TURRET_SLEW, TURRET_SLEW);
            power = lastTurretPower + delta;
            lastTurretPower = power;

            int pos = turret.getCurrentPosition();
            if ((power > 0 && pos >= TURRET_MAX_TICKS) ||
                    (power < 0 && pos <= TURRET_MIN_TICKS)) {
                stopTurret();
                return;
            }

            turret.setPower(power);
        } else {
            stopTurret();
        }
    }

    private void stopTurret() {
        turret.setPower(0);
        lastTurretPower = 0;
        lastError = 0;
        filteredError = 0;
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

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
