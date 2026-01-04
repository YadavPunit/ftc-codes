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
@TeleOp(name = "test12")
public class test12 extends OpMode {

    private Follower follower;
    private TelemetryManager telemetryM;

    private DcMotor intake;
    private DcMotorEx shooterL, shooterR;
    private Servo leftRamp, rightRamp, hood;
    private Limelight3A limelight;

    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 60.0;
    static final double SHOOTER_kI = 0.0;
    static final double SHOOTER_kD = 6.0;
    static final double SHOOTER_kF = 12.0;

    static final double ALIGN_KP = 0.03;
    static final double ALIGN_KD = 0.001;

    // 🔥 ALIGN TARGET = TY = -2
    static final double TY_ALIGN_OFFSET = -2.0;
    static final double ALIGN_DEADBAND = 0.3;

    // ✅ FIX: must be large enough to allow turning
    static final double MAX_ALIGN_ERROR = 8.0;

    static final double MAX_TURN = 0.5;

    static final double INTAKE_POWER = -0.5;
    static final double SLOW_MODE = 0.5;

    static final double TX_SHORT_MIN = -4.97;
    static final double TX_SHORT_MAX = 14.0;
    static final double RPM_SHORT_MIN = 2850;
    static final double RPM_SHORT_MAX = 3500;
    static final double HOOD_SHORT_MIN = 0.95;
    static final double HOOD_SHORT_MAX = 0.45;

    static final double TX_LONG_MIN = 17.0;
    static final double TX_LONG_MAX = 17.9;
    static final double RPM_LONG_MIN = 4150;
    static final double RPM_LONG_MAX = 4400;
    static final double HOOD_LONG_MIN = 0.3;
    static final double HOOD_LONG_MAX = 0.3;

    static final double RAMP_UP = 0.65;
    static final double RAMP_DOWN = 0.37;

    private boolean intakeToggle = false;
    private boolean rampToggle = false;
    private boolean prevX = false, prevSquare = false;

    private double lastAlignError = 0;
    private boolean limelightStarted = false;
    private long startTime;

    private double lastLLTime = 0;
    private double lastShooterTime = 0;

    static final double LL_DT = 0.04;
    static final double SHOOTER_DT = 0.05;

    private double cachedTx = 0;
    private double cachedTy = 0;
    private boolean cachedHasTarget = false;

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

        hood = hardwareMap.get(Servo.class, "hood");
        leftRamp = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");

        setRamp(false);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        startTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        double now = getRuntime();

        follower.update();
        telemetryM.update();

        if (!limelightStarted && System.currentTimeMillis() - startTime > 1000) {
            limelight.start();
            limelightStarted = true;
        }

        if (limelightStarted && now - lastLLTime > LL_DT) {
            lastLLTime = now;
            LLResult ll = limelight.getLatestResult();
            if (ll != null && ll.isValid()) {
                cachedTx = ll.getTx();
                cachedTy = ll.getTy();
                cachedHasTarget = true;
            } else {
                cachedHasTarget = false;
            }
        }

        boolean alignBot = gamepad1.left_bumper;
        boolean slowMode = gamepad1.right_bumper;
        boolean shootMode = gamepad2.left_bumper;

        double speedMul = slowMode ? SLOW_MODE : 1.0;

        double forward = -gamepad1.left_stick_y * speedMul;
        double strafe  = -gamepad1.left_stick_x * speedMul;
        double turn    = -gamepad1.right_stick_x * speedMul;

        // ===== BOT ALIGN (WORKING) =====
        if (alignBot && cachedHasTarget) {

            double error = cachedTy - TY_ALIGN_OFFSET;
            error = clamp(error, -MAX_ALIGN_ERROR, MAX_ALIGN_ERROR);

            if (Math.abs(error) < ALIGN_DEADBAND) error = 0;

            double derivative = error - lastAlignError;
            lastAlignError = error;

            turn = -(ALIGN_KP * error + ALIGN_KD * derivative);
            turn = clamp(turn, -MAX_TURN, MAX_TURN);

        } else {
            lastAlignError = 0;
        }

        follower.setTeleOpDrive(forward, strafe, turn, true);

        if (gamepad2.cross && !prevX) intakeToggle = !intakeToggle;
        if (gamepad2.square && !prevSquare) rampToggle = !rampToggle;
        prevX = gamepad2.cross;
        prevSquare = gamepad2.square;

        intake.setPower(intakeToggle ? INTAKE_POWER : 0);
        setRamp(rampToggle);

        if (shootMode && cachedHasTarget && now - lastShooterTime > SHOOTER_DT) {
            lastShooterTime = now;
            updateAutoShot(cachedTx);
        } else if (!shootMode) {
            shooterL.setVelocity(0);
            shooterR.setVelocity(0);
        }
    }

    private void updateAutoShot(double tx) {

        double targetRPM, hoodPos;

        if (tx <= TX_SHORT_MAX) {
            targetRPM = lerp(TX_SHORT_MIN, TX_SHORT_MAX,
                    RPM_SHORT_MIN, RPM_SHORT_MAX, tx);
            hoodPos = lerp(TX_SHORT_MIN, TX_SHORT_MAX,
                    HOOD_SHORT_MIN, HOOD_SHORT_MAX, tx);
        } else {
            targetRPM = lerp(TX_LONG_MIN, TX_LONG_MAX,
                    RPM_LONG_MIN, RPM_LONG_MAX, tx);
            hoodPos = lerp(TX_LONG_MIN, TX_LONG_MAX,
                    HOOD_LONG_MIN, HOOD_LONG_MAX, tx);
        }

        double leftRPM  = (shooterL.getVelocity() / TICKS_PER_REV) * 60.0;
        double rightRPM = (shooterR.getVelocity() / TICKS_PER_REV) * 60.0;
        double finalRPM = (leftRPM + rightRPM + targetRPM) / 3.0;

        shooterL.setVelocity(rpmToTicks(finalRPM));
        shooterR.setVelocity(rpmToTicks(finalRPM));
        hood.setPosition(clamp(hoodPos, 0.2, 1.0));
    }

    private double rpmToTicks(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    private double lerp(double x1, double x2, double y1, double y2, double x) {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void setRamp(boolean up) {
        double pos = up ? RAMP_UP : RAMP_DOWN;
        leftRamp.setPosition(pos);
        rightRamp.setPosition(1.0 - pos);
    }
}
