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
@TeleOp(name = "test6")
public class test6 extends OpMode {

    // ================= PEDRO =================
    private Follower follower;
    public static Pose startingPose;
    private TelemetryManager telemetryM;

    // ================= HARDWARE =================
    private DcMotor intake;
    private DcMotorEx shooting, shooting1;
    private Servo leftRamp, rightRamp, hood;
    private Limelight3A limelight;

    // ================= CONSTANTS =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 60.0;
    static final double SHOOTER_kI = 0.0;
    static final double SHOOTER_kD = 6.0;
    static final double SHOOTER_kF = 12.0;

    static final double SHOOTER_ACCEL_RPM_PER_SEC = 2500; // 🔥 acceleration

    static final double ALIGN_KP = 0.03;
    static final double ALIGN_KD = 0.001;
    static final double TY_OFFSET = 8;

    static final double RAMP_MIN = 0.35;
    static final double RAMP_MAX = 0.70;
    static final double HOOD_MIN = 0.30;
    static final double HOOD_MAX = 1.00;

    static final double INTAKE_POWER = -0.8;

    // ================= STATE =================
    private double lastAlignError = 0;

    private boolean intakeToggle = false;
    private boolean rampToggle   = false;

    private boolean prevA = false, prevX = false;

    private boolean limelightStarted = false;
    private long startTime;

    // Shooter ramp state
    private double currentShooterRPM = 0;
    private double lastShooterTime   = 0;

    // ================= INIT =================
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(
                startingPose == null ? new Pose(0, 0, 0) : startingPose
        );

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake    = hardwareMap.get(DcMotor.class, "intake");
        shooting  = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooting1 = hardwareMap.get(DcMotorEx.class, "right_shooting");

        leftRamp  = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");
        hood      = hardwareMap.get(Servo.class, "hood");

        shooting1.setDirection(DcMotor.Direction.REVERSE);

        shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooting.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF
        );
        shooting1.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF
        );

        setRampPosition(0.37);
        setHoodPosition(0.50);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        startTime = System.currentTimeMillis();
        lastShooterTime = getRuntime();
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

        LLResult ll = limelightStarted ? limelight.getLatestResult() : null;
        boolean hasTarget = ll != null && ll.isValid();
        boolean lbMode = gamepad1.left_bumper && hasTarget;

        // ================= BOT ALIGN =================
        if (lbMode) {
            double error = -(ll.getTy() + TY_OFFSET);
            double derivative = error - lastAlignError;
            lastAlignError = error;
            turn = (ALIGN_KP * error) + (ALIGN_KD * derivative);
        } else {
            lastAlignError = 0;
        }

        follower.setTeleOpDrive(forward, strafe, turn, true);

        // ================= TOGGLES =================
        if (gamepad2.cross && !prevA) intakeToggle = !intakeToggle;
        if (gamepad2.square && !prevX) rampToggle = !rampToggle;

        prevA = gamepad2.cross;
        prevX = gamepad2.square;

        // ================= INTAKE =================
        intake.setPower(intakeToggle ? INTAKE_POWER : 0);

        // ================= SHOOTER + AUTO RAMP =================
        if (lbMode) {
            updateShooterHoodRamp(ll.getTx());
        } else {
            currentShooterRPM = 0; // reset ramp
            shooting.setVelocity(0);
            shooting1.setVelocity(0);
            setRampPosition(rampToggle ? 0.65 : 0.37);
        }

        telemetryM.debug("LB Mode", lbMode);
        telemetryM.debug("Shooter RPM", currentShooterRPM);
    }

    // ================= HELPERS =================
    private double rpmToTicks(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    private double map(double x, double inMin, double inMax,
                       double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin)
                / (inMax - inMin) + outMax;
    }

    private void setRampPosition(double pos) {
        pos = Math.max(RAMP_MIN, Math.min(RAMP_MAX, pos));
        leftRamp.setPosition(pos);
        rightRamp.setPosition(1.0 - pos);
    }

    private void setHoodPosition(double pos) {
        pos = Math.max(HOOD_MIN, Math.min(HOOD_MAX, pos));
        hood.setPosition(pos);
    }

    // ================= SHOOTER ACCELERATION =================
    private double rampRPM(double targetRPM) {

        double now = getRuntime();
        double dt = now - lastShooterTime;
        lastShooterTime = now;

        double maxStep = SHOOTER_ACCEL_RPM_PER_SEC * dt;

        if (targetRPM > currentShooterRPM) {
            currentShooterRPM = Math.min(currentShooterRPM + maxStep, targetRPM);
        } else {
            currentShooterRPM = Math.max(currentShooterRPM - maxStep, targetRPM);
        }

        return currentShooterRPM;
    }

    // ================= LIMELIGHT AUTO =================
    private void updateShooterHoodRamp(double tx) {

        tx = Math.max(-4.97, Math.min(14.0, tx));

        double targetRPM = map(tx, -4.97, 14.0, 3100, 4050);
        double hoodPos   = map(tx, -4.97, 14.0, 0.95, 0.45);

        double smoothRPM = rampRPM(targetRPM);

        shooting.setVelocity(rpmToTicks(smoothRPM));
        shooting1.setVelocity(rpmToTicks(smoothRPM));

        setHoodPosition(hoodPos);
    }
}
