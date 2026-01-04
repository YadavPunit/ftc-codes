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
@TeleOp(name = "test4")
public class test4 extends OpMode {

    // ================= Drive =================
    private Follower follower;
    private TelemetryManager telemetryM;

    // ================= Hardware =================
    private DcMotor intake;
    private DcMotorEx shooting, shooting1;
    private Servo ramp, hood;
    private Limelight3A limelight;

    // ================= Shooter (MEASURED) =================
    private static final double MAX_RPM = 2300.0;
    private static final double MIN_RPM = 1400.0;
    private static final double RPM_BOOST = 300.0;

    // ================= Limelight Mapping =================
    private static final double TX_NEAR = 4.5;
    private static final double TX_FAR  = 18.0;

    // ================= Hood Mapping =================
    private static final double HOOD_NEAR = 0.80;
    private static final double HOOD_FAR  = 0.35;

    // ================= BOT ALIGN PID (TY) =================
    private static final double ALIGN_kP = 0.03;
    private static final double ALIGN_kI = 0.0;
    private static final double ALIGN_kD = 0.002;
    private static final double TARGET_TY = -8.0;
    private static final double MAX_TURN = 0.35;

    private double alignIntegral = 0;
    private double alignLastError = 0;

    // ================= Drive State =================
    private double driveForward = 0;
    private double driveStrafe = 0;
    private boolean slowMode = false;
    private final double slowMultiplier = 0.5;

    private boolean intakeOn = false;
    private boolean prevSquare = false;
    private boolean prevLeftBumper = false;

    public static Pose startingPose;
    private Supplier<PathChain> pathChain;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        intake    = hardwareMap.get(DcMotor.class, "intake");
        shooting  = hardwareMap.get(DcMotorEx.class, "shooting");
        shooting1 = hardwareMap.get(DcMotorEx.class, "shooting1");
        ramp      = hardwareMap.get(Servo.class, "ramp1");
        hood      = hardwareMap.get(Servo.class, "hood");

        shooting.setDirection(DcMotor.Direction.REVERSE);
        shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            applyLimelightBotAlignAndShoot();
            intake.setPower(-0.4);
        }

        if (gamepad2.triangle) ramp.setPosition(0.6);
        else if (gamepad2.cross) ramp.setPosition(0.92);

        if (!leftBumper && prevLeftBumper) {
            shooting.setVelocity(0);
            shooting1.setVelocity(0);
            intake.setPower(0);
            alignIntegral = 0;
            alignLastError = 0;
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

        double mult = slowMode ? slowMultiplier : 1.0;

        follower.setTeleOpDrive(
                driveForward * mult,
                driveStrafe * mult,
                turn * mult,
                true
        );
    }

    // ================= LIMELIGHT → BOT ALIGN + SHOOT =================
    private void applyLimelightBotAlignAndShoot() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        double ty = result.getTy();
        double tx = Math.abs(result.getTx());

        // ---------- BOT ALIGN PID ----------
        double error = TARGET_TY - ty;

        alignIntegral += error;
        double derivative = error - alignLastError;
        alignLastError = error;

        double turn =
                (ALIGN_kP * error) +
                        (ALIGN_kI * alignIntegral) +
                        (ALIGN_kD * derivative);

        turn = clip(turn, -MAX_TURN, MAX_TURN);

        follower.setTeleOpDrive(
                0,   // lock forward
                0,   // lock strafe
                turn,
                true
        );

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
        telemetryM.debug("TY", ty);
        telemetryM.debug("TargetRPM", targetRPM);
        telemetryM.debug("ShooterRPM", shooting.getVelocity());
        telemetryM.debug("Hood", hoodPos);
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
