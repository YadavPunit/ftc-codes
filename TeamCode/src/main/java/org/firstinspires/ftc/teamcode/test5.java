package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "test5")
public class test5 extends OpMode {

    // ================= Drive =================
    private Follower follower;
    private TelemetryManager telemetryM;

    // ================= Hardware =================
    private DcMotor intake;
    private DcMotorEx shooting, shooting1;
    private Servo ramp, hood;
    private Limelight3A limelight;

    // ================= Shooter RPM (MEASURED) =================
    private static final double MIN_RPM = 1400;
    private static final double MAX_RPM = 2600;

    // ================= Limelight Mapping =================
    private static final double TX_NEAR = 4.5;
    private static final double TX_FAR  = 18.0;

    private static final double HOOD_NEAR = 0.80;
    private static final double HOOD_FAR  = 0.28;

    // ================= BOT ALIGN PID (TY) =================
    private static final double TARGET_TY = -8.0;
    private static final double kP = 0.03;
    private static final double kI = 0.0;
    private static final double kD = 0.002;
    private static final double MAX_TURN = 0.35;

    private double integral = 0;
    private double lastError = 0;

    private boolean prevLeftBumper = false;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

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

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double manualTurn = -gamepad1.right_stick_x;

        boolean leftBumper = gamepad1.left_bumper;

        if (leftBumper) {
            // 🔥 Assisted shooting mode
            double turnPID = limelightAlignAndShoot();
            follower.setTeleOpDrive(forward, strafe, turnPID, true);
            intake.setPower(-0.5);
        } else {
            // Normal drive
            follower.setTeleOpDrive(forward, strafe, manualTurn, true);
        }

        // ================= Ramp Control =================
        if (gamepad2.square)   ramp.setPosition(0.6);
        if (gamepad2.triangle) ramp.setPosition(0.92);

        // ================= Stop on release =================
        if (!leftBumper && prevLeftBumper) {
            shooting.setVelocity(0);
            shooting1.setVelocity(0);
            intake.setPower(0);
            integral = 0;
            lastError = 0;
        }

        prevLeftBumper = leftBumper;
        telemetryM.update();
    }

    // ================= LIMELIGHT BOT ALIGN + SHOOT =================
    private double limelightAlignAndShoot() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0;

        double ty = result.getTy();
        double tx = Math.abs(result.getTx());

        // ----- BOT ROTATION PID -----
        double error = TARGET_TY - ty;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double turn =
                (kP * error) +
                        (kI * integral) +
                        (kD * derivative);

        turn = clip(turn, -MAX_TURN, MAX_TURN);

        // ----- TX → RPM -----
        double targetRPM = map(tx, TX_NEAR, TX_FAR, MIN_RPM, MAX_RPM);
        targetRPM = clip(targetRPM, MIN_RPM, MAX_RPM);

        shooting.setVelocity(targetRPM);
        shooting1.setVelocity(targetRPM);

        // ----- TX → HOOD -----
        double hoodPos = map(tx, TX_NEAR, TX_FAR, HOOD_NEAR, HOOD_FAR);
        hoodPos = clip(hoodPos, HOOD_FAR, HOOD_NEAR);
        hood.setPosition(hoodPos);

        telemetryM.debug("TX", tx);
        telemetryM.debug("TY", ty);
        telemetryM.debug("TargetRPM", targetRPM);
        telemetryM.debug("ShooterRPM", shooting.getVelocity());
        telemetryM.debug("Hood", hoodPos);

        return turn;
    }

    // ================= UTIL =================
    private double map(double x, double inMin, double inMax,
                       double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin)
                / (inMax - inMin) + outMin;
    }

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}