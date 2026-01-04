package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "test3")
public class test3 extends OpMode {

    // ================= Drive =================
    private Follower follower;
    private TelemetryManager telemetryM;

    // ================= Hardware =================
    private DcMotor intake;
    private DcMotorEx shooting, shooting1;
    private DcMotorEx turret;
    private Servo ramp, hood;
    private Limelight3A limelight;

    // ================= Shooter (MEASURED) =================
    private static final double MAX_RPM = 2300.0;
    private static final double MIN_RPM = 1400.0;
    private static final double RPM_BOOST = 300.0;   // small boost only

    // ================= Limelight Mapping =================
    private static final double TX_NEAR = 4.5;
    private static final double TX_FAR  = 18.0;

    // ================= Hood Mapping =================
    private static final double HOOD_NEAR = 0.80;
    private static final double HOOD_FAR  = 0.35;

    // ================= Turret Limits =================
    private static final int TURRET_LEFT_LIMIT  = -100;
    private static final int TURRET_RIGHT_LIMIT =  100;

    // ================= Turret PID =================
    private static final double TURRET_kP = 0.015;
    private static final double TURRET_kI = 0.0;
    private static final double TURRET_kD = 0.0015;
    private static final double TURRET_MAX_POWER = 0.35;
    private static final double TURRET_DEADBAND = 0.15;

    private double turretIntegral = 0;
    private double turretLastError = 0;

    // ================= Drive State =================
    private double driveForward = 0;
    private double driveStrafe = 0;
    private boolean slowMode = false;
    private final double slowMultiplier = 0.5;
    private boolean automatedDrive = false;

    private boolean intakeOn = false;
    private boolean prevSquare = false;
    private boolean prevLeftBumper = false;

    public static Pose startingPose;
    private Supplier<PathChain> pathChain;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        startingPose = new Pose(38.58, 33.64, Math.toRadians(90));
        follower.setStartingPose(startingPose);
        follower.update();

        intake    = hardwareMap.get(DcMotor.class, "intake");
        shooting  = hardwareMap.get(DcMotorEx.class, "shooting");
        shooting1 = hardwareMap.get(DcMotorEx.class, "shooting1");
        turret    = hardwareMap.get(DcMotorEx.class, "turret");
        ramp      = hardwareMap.get(Servo.class, "ramp1");
        hood      = hardwareMap.get(Servo.class, "hood");

        shooting.setDirection(DcMotor.Direction.REVERSE);

        // Encoder-based velocity
        shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ramp.setPosition(0.92);
        hood.setPosition(0.92);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
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

        handleToggles();
        handleDriving();

        boolean leftBumper = gamepad1.left_bumper;

        if (leftBumper) {
            applyLimelightAimAlignShoot();
            intake.setPower(-0.4);
        }

        if (!leftBumper && prevLeftBumper) {
            shooting.setVelocity(0);
            shooting1.setVelocity(0);
            turret.setPower(0);
            intake.setPower(0);
            turretIntegral = 0;
            turretLastError = 0;
        }

        prevLeftBumper = leftBumper;

        telemetryM.update();
        updateTelemetry();
    }

    // ================= Toggles =================
    private void handleToggles() {

        if (gamepad2.square && !prevSquare) intakeOn = !intakeOn;
        prevSquare = gamepad2.square;
        intake.setPower(intakeOn ? -0.8 : 0);

        if (gamepad1.right_bumper) slowMode = !slowMode;
    }

    // ================= Driving =================
    private void handleDriving() {

        driveForward = -gamepad1.left_stick_y;
        driveStrafe  = -gamepad1.left_stick_x;
        double turn  = -gamepad1.right_stick_x;

        if (!automatedDrive) {
            double mult = slowMode ? slowMultiplier : 1.0;
            follower.setTeleOpDrive(
                    driveForward * mult,
                    driveStrafe * mult,
                    turn * mult,
                    true
            );
        }
    }

    // ================= LIMELIGHT → TURRET + RPM + HOOD =================
    private void applyLimelightAimAlignShoot() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        double ty = result.getTy();
        double tx = Math.abs(result.getTx());

        // ---------- TURRET PID ----------
        double error = -ty;

        if (Math.abs(error) < TURRET_DEADBAND) {
            turret.setPower(0);
        } else {
            turretIntegral += error;
            double derivative = error - turretLastError;
            turretLastError = error;

            double output =
                    (TURRET_kP * error) +
                            (TURRET_kI * turretIntegral) +
                            (TURRET_kD * derivative);

            output = clip(output, -TURRET_MAX_POWER, TURRET_MAX_POWER);

            int pos = turret.getCurrentPosition();
            if ((output < 0 && pos <= TURRET_LEFT_LIMIT) ||
                    (output > 0 && pos >= TURRET_RIGHT_LIMIT)) {
                output = 0;
            }

            turret.setPower(output);
        }

        // ---------- TX → HOOD ----------
        double hoodPos = map(tx, TX_NEAR, TX_FAR, HOOD_NEAR, HOOD_FAR);
        hoodPos = clip(hoodPos, HOOD_FAR, HOOD_NEAR);
        hood.setPosition(hoodPos);

        // ---------- TX → RPM ----------
        double baseRPM = map(tx, TX_NEAR, TX_FAR, MIN_RPM, MAX_RPM);
        baseRPM = clip(baseRPM, MIN_RPM, MAX_RPM);

        double targetRPM =
                ramp.getPosition() < 0.7
                        ? clip(baseRPM + RPM_BOOST, MIN_RPM, MAX_RPM)
                        : baseRPM;

        shooting.setVelocity(targetRPM);
        shooting1.setVelocity(targetRPM);

        // ---------- TELEMETRY ----------
        telemetryM.debug("TX", tx);
        telemetryM.debug("TargetRPM", targetRPM);
        telemetryM.debug("ShooterRPM", shooting.getVelocity());
        telemetryM.debug("Hood", hoodPos);
        telemetryM.debug("TurretEnc", turret.getCurrentPosition());
    }

    private void updateTelemetry() {
        telemetryM.debug("Pose", follower.getPose());
    }

    // ================= Utilities =================
    private double map(double x, double inMin, double inMax,
                       double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin)
                / (inMax - inMin) + outMin;
    }

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
