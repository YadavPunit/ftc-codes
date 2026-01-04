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
@TeleOp(name = "test8")
public class test8 extends OpMode {

    // ================= HARDWARE =================
    private Follower follower;
    private TelemetryManager telemetryM;

    private DcMotor intake;
    private DcMotorEx shooterL, shooterR;
    private Servo leftRamp, rightRamp, hood;
    private Limelight3A limelight;

    // ================= CONSTANTS =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 60, SHOOTER_kI = 0, SHOOTER_kD = 6, SHOOTER_kF = 12;
    static final double SHOOTER_ACCEL_RPM_PER_SEC = 2500;

    static final double ALIGN_KP = 0.03;
    static final double ALIGN_KD = 0.001;

    static final double INTAKE_POWER = -0.8;

    // ---- DISTANCE MAPPING (TX) ----
    static final double TX_NEAR = -5.0;
    static final double TX_FAR  = 14.0;

    static final double RPM_NEAR = 3100;
    static final double RPM_FAR  = 4050;

    static final double HOOD_NEAR = 0.95;
    static final double HOOD_FAR  = 0.45;

    static final double RAMP_DOWN = 0.37;
    static final double RAMP_UP   = 0.65;

    // ================= STATE =================
    private boolean intakeToggle = false;
    private boolean rampToggle   = false;
    private boolean prevA = false, prevX = false;

    private double lastAlignError = 0;
    private double currentRPM = 0;
    private double lastTime = 0;

    private boolean limelightStarted = false;
    private long startTime;

    // ================= INIT =================
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooterL = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterR = hardwareMap.get(DcMotorEx.class, "right_shooting");

        leftRamp  = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");
        hood      = hardwareMap.get(Servo.class, "hood");

        shooterR.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterR.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        setRamp(RAMP_DOWN);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        startTime = System.currentTimeMillis();
        lastTime = getRuntime();
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        if (!limelightStarted && System.currentTimeMillis() - startTime > 1000) {
            limelight.start();
            limelightStarted = true;
        }

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;

        boolean lbPressed = gamepad1.left_bumper;

        LLResult ll = limelightStarted ? limelight.getLatestResult() : null;
        boolean hasTarget =
                ll != null &&
                        ll.isValid() &&
                        !Double.isNaN(ll.getTx()) &&
                        !Double.isNaN(ll.getTy());

        // ===== BOT ALIGN (USING TY – CAMERA ROTATED) =====
        if (lbPressed && hasTarget) {
            double error = ll.getTy(); // 🔥 CORRECT AXIS
            double derivative = error - lastAlignError;
            lastAlignError = error;

            turn = -(ALIGN_KP * error + ALIGN_KD * derivative);
        } else {
            lastAlignError = 0;
        }

        follower.setTeleOpDrive(forward, strafe, turn, true);


        if (gamepad2.cross && !prevA) intakeToggle = !intakeToggle;
        if (gamepad2.square && !prevX) rampToggle = !rampToggle;
        prevA = gamepad2.cross;
        prevX = gamepad2.square;

        // ===== INTAKE =====
        intake.setPower(intakeToggle ? INTAKE_POWER : 0);

        // ===== RAMP =====
        setRamp(rampToggle ? RAMP_UP : RAMP_DOWN);

        // ===== SHOOTER + HOOD (TX BASED) =====
        if (lbPressed && hasTarget) {
            updateAutoShot(ll.getTx());
        } else if (!lbPressed) {
            currentRPM = 0;
            shooterL.setVelocity(0);
            shooterR.setVelocity(0);
        }
    }

    // ================= AUTO SHOOT =================
    private void updateAutoShot(double tx) {

        tx = clamp(tx, TX_NEAR, TX_FAR);

        double targetRPM = lerp(TX_NEAR, TX_FAR, RPM_NEAR, RPM_FAR, tx);
        double hoodPos   = lerp(TX_NEAR, TX_FAR, HOOD_NEAR, HOOD_FAR, tx);

        double smoothRPM = rampRPM(targetRPM);

        shooterL.setVelocity(rpmToTicks(smoothRPM));
        shooterR.setVelocity(rpmToTicks(smoothRPM));
        hood.setPosition(clamp(hoodPos, 0.30, 1.00));
    }

    // ================= HELPERS =================
    private double rampRPM(double target) {
        double now = getRuntime();
        double dt = Math.min(0.05, now - lastTime);
        lastTime = now;

        double step = SHOOTER_ACCEL_RPM_PER_SEC * dt;

        if (target > currentRPM)
            currentRPM = Math.min(currentRPM + step, target);
        else
            currentRPM = Math.max(currentRPM - step, target);

        return currentRPM;
    }

    private double lerp(double x1, double x2, double y1, double y2, double x) {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double rpmToTicks(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    private void setRamp(double pos) {
        pos = clamp(pos, 0.35, 0.70);
        leftRamp.setPosition(pos);
        rightRamp.setPosition(1.0 - pos);
    }
}
