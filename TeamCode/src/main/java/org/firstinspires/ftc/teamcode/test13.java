package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "test13_manualShoot")
public class test13 extends OpMode {

    // ================= CORE =================
    private Follower follower;
    private TelemetryManager telemetryM;

    // ================= MOTORS =================
    private DcMotor intake;
    private DcMotorEx shooterL, shooterR;

    // ================= SERVOS =================
    private Servo hood;
    private Servo leftRamp, rightRamp;
    private Servo leftLift, rightLift;

    // ================= CONSTANTS =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 60.0;
    static final double SHOOTER_kI = 0.0;
    static final double SHOOTER_kD = 6.0;
    static final double SHOOTER_kF = 12.0;

    static final double INTAKE_POWER = -0.5;
    static final double SLOW_MODE = 0.5;

    // ================= RAMP VALUES =================
    static final double RAMP_UP_LEFT = 0.3;
    static final double RAMP_UP_RIGHT = 0.7;
    static final double RAMP_DOWN_LEFT = 0.6;
    static final double RAMP_DOWN_RIGHT = 0.4;

    // ================= LIFT VALUES =================
    static final double LIFT_UP_LEFT = 1.0;
    static final double LIFT_UP_RIGHT = 0.12;
    static final double LIFT_DOWN_LEFT = 0.2;
    static final double LIFT_DOWN_RIGHT = 0.8;

    // ================= SHOOT PRESETS =================
    static final double SHORT_RPM = 2950;
    static final double SHORT_HOOD = 0.95;

    static final double LONG_RPM = 3790;
    static final double LONG_HOOD = 0.28;

    // ================= STATE =================
    private boolean intakeToggle = false;
    private boolean rampToggle = false;

    private boolean shortShot = false;
    private boolean longShot = false;

    private boolean prevX = false, prevSquare = false;
    private boolean prevTriangle = false, prevCircle = false;

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

        shooterL.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterR.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(0.2, 0.8);
        hood.setPosition(0.5);

        leftRamp = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");

        leftLift = hardwareMap.get(Servo.class, "left_lift");
        rightLift = hardwareMap.get(Servo.class, "right_lift");

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

        // ===== DRIVE =====
        boolean slowMode = gamepad1.right_bumper;
        double speedMul = slowMode ? SLOW_MODE : 1.0;

        double forward = -gamepad1.left_stick_y * speedMul;
        double strafe  = -gamepad1.left_stick_x * speedMul;
        double turn    = -gamepad1.right_stick_x * speedMul;

        follower.setTeleOpDrive(forward, strafe, turn, true);

        // ===== LIFT (GAMEPAD 1) =====
        if (gamepad1.dpad_up) {
            setLift(true);
        } else if (gamepad1.dpad_down) {
            setLift(false);
        }

        // ===== TOGGLES (GAMEPAD 2) =====
        if (gamepad2.cross && !prevX) intakeToggle = !intakeToggle;
        if (gamepad2.square && !prevSquare) rampToggle = !rampToggle;

        if (gamepad2.triangle && !prevTriangle) {
            shortShot = !shortShot;
            longShot = false;
        }

        if (gamepad2.circle && !prevCircle) {
            longShot = !longShot;
            shortShot = false;
        }

        prevX = gamepad2.cross;
        prevSquare = gamepad2.square;
        prevTriangle = gamepad2.triangle;
        prevCircle = gamepad2.circle;

        intake.setPower(intakeToggle ? INTAKE_POWER : 0);
        setRamp(rampToggle);

        // ===== SHOOTER =====
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

    // ================= HELPERS =================
    private void runShooterRPM(double rpm) {
        double ticks = (rpm / 60.0) * TICKS_PER_REV;
        shooterL.setVelocity(ticks);
        shooterR.setVelocity(ticks);
    }

    private void setRamp(boolean up) {
        if (up) {
            leftRamp.setPosition(RAMP_UP_LEFT);
            rightRamp.setPosition(RAMP_UP_RIGHT);
        } else {
            leftRamp.setPosition(RAMP_DOWN_LEFT);
            rightRamp.setPosition(RAMP_DOWN_RIGHT);
        }
    }

    private void setLift(boolean up) {
        if (up) {
            leftLift.setPosition(LIFT_UP_LEFT);
            rightLift.setPosition(LIFT_UP_RIGHT);
        } else {
            leftLift.setPosition(LIFT_DOWN_LEFT);
            rightLift.setPosition(LIFT_DOWN_RIGHT);
        }
    }
}
