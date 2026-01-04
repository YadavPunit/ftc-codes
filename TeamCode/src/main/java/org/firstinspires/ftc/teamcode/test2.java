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
@TeleOp(name = "test2")
public class test2 extends OpMode {

    // ===== Drive =====
    private Follower follower;
    private TelemetryManager telemetryM;

    // ===== Hardware =====
    private DcMotor intake;
    private DcMotorEx shooting;
    private DcMotorEx shooting1;
    private Servo ramp;
    private Servo hood;
    private Limelight3A limelight;

    // ===== Shooter Power Adjust =====
    private static final double SHOOTER_STEP = 0.05;
    private static final double SHOOTER_MIN  = 0.55;
    private static final double SHOOTER_MAX  = 1.0;

    private double shooterPower = 0.80;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    // ===== Hood Manual Control =====
    private static final double HOOD_STEP = 0.1;
    private static final double HOOD_MIN  = 0.3;
    private static final double HOOD_MAX  = 1.0;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    // ===== PID (TURN USING TY) =====
    private static final double kP = 0.03;
    private static final double kI = 0.0;
    private static final double kD = 0.002;
    private static final double TARGET_TY = -8.0;

    private double turnIntegral = 0;
    private double lastError = 0;

    // ===== Drive State =====
    private double driveForward = 0;
    private double driveStrafe = 0;
    private boolean slowMode = false;
    private double slowMultiplier = 0.5;

    private boolean intakeOn = false;
    private boolean prevSquare = false;
    private boolean prevSquare1 = false;
    private boolean prevCircle = false;
    private boolean shoot1 = false;
    private boolean shoot2 = false;
    private boolean prevLeftBumper = false;

    public static Pose startingPose;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        startingPose = new Pose(38.58, 33.64, Math.toRadians(90));
        follower.setStartingPose(startingPose);

        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooting = hardwareMap.get(DcMotorEx.class, "shooting");
        shooting1= hardwareMap.get(DcMotorEx.class, "shooting1");
        ramp     = hardwareMap.get(Servo.class, "ramp1");
        hood     = hardwareMap.get(Servo.class, "hood");

        shooting.setDirection(DcMotor.Direction.REVERSE);

        // POWER MODE
        shooting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooting1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ramp.setPosition(0.92);
        hood.setPosition(0.92);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        follower.update();

        handleShooterPowerAdjust();
        handleToggles();
        handleDriving();
        handleHoodManualControl();

        boolean leftBumper = gamepad1.left_bumper;

        if (leftBumper) {
            applyLimelightAimAlignShoot();
            intake.setPower(-0.6);
        }

        if (gamepad2.triangle) ramp.setPosition(0.6);
        else if (gamepad2.cross) ramp.setPosition(0.92);

        if (!leftBumper && prevLeftBumper) {
            shooting.setPower(0);
            shooting1.setPower(0);
            intake.setPower(0);
            turnIntegral = 0;
            lastError = 0;
        }

        prevLeftBumper = leftBumper;

        telemetryM.debug("ShooterPower", shooterPower);
        telemetryM.update();
    }

    // ===== DPAD LEFT / RIGHT → SHOOTER POWER =====
    private void handleShooterPowerAdjust() {

        boolean dpadLeft  = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        if (dpadRight && !prevDpadRight) {
            shooterPower += SHOOTER_STEP;
        }
        if (dpadLeft && !prevDpadLeft) {
            shooterPower -= SHOOTER_STEP;
        }

        shooterPower = clip(shooterPower, SHOOTER_MIN, SHOOTER_MAX);

        prevDpadLeft = dpadLeft;
        prevDpadRight = dpadRight;
    }

    // ===== Manual Hood Control =====
    private void handleHoodManualControl() {

        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        double hoodPos = hood.getPosition();

        if (dpadUp && !prevDpadUp) hoodPos += HOOD_STEP;
        if (dpadDown && !prevDpadDown) hoodPos -= HOOD_STEP;

        hoodPos = clip(hoodPos, HOOD_MIN, HOOD_MAX);
        hood.setPosition(hoodPos);

        prevDpadUp = dpadUp;
        prevDpadDown = dpadDown;
    }

    private void handleToggles() {

        if (gamepad2.square && !prevSquare) intakeOn = !intakeOn;
        prevSquare = gamepad2.square;
        intake.setPower(intakeOn ? -0.55 : 0);

        if (gamepad1.square && !prevSquare1) shoot1 = !shoot1;
        prevSquare1 = gamepad1.square;

        if (shoot1) {
            shooting.setPower(shooterPower);
            shooting1.setPower(shooterPower);
        }

        if (gamepad1.circle && !prevCircle) shoot2 = !shoot2;
        prevCircle = gamepad1.circle;

        if (shoot2) {
            shooting.setPower(shooterPower);
            shooting1.setPower(shooterPower);
            intake.setPower(-0.55);
        }

        if (!shoot1 && !shoot2 && !gamepad1.left_bumper) {
            shooting.setPower(0);
            shooting1.setPower(0);
        }

        if (gamepad1.right_bumper) slowMode = !slowMode;
    }

    private void handleDriving() {

        driveForward = -gamepad1.left_stick_y;
        driveStrafe  = -gamepad1.left_stick_x;
        double turn  = gamepad1.left_bumper ? 0 : -gamepad1.right_stick_x;

        double mult = slowMode ? slowMultiplier : 1.0;
        follower.setTeleOpDrive(
                driveForward * mult,
                driveStrafe * mult,
                turn * mult,
                true
        );
    }

    // ===== LIMELIGHT ALIGN =====
    private void applyLimelightAimAlignShoot() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        double ty = result.getTy();
        double error = TARGET_TY - ty;

        turnIntegral += error;
        double derivative = error - lastError;
        lastError = error;

        double turnPower =
                (kP * error) +
                        (kI * turnIntegral) +
                        (kD * derivative);

        turnPower = clip(turnPower, -0.35, 0.35);

        follower.setTeleOpDrive(
                driveForward,
                driveStrafe,
                turnPower,
                true
        );
    }

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
