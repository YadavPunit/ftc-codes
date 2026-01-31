package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "test29_turret_tuning", group = "Tuning")
public class test29_turret_tuning extends OpMode {

    // ================= HARDWARE =================
    private Follower follower;
    private DcMotorEx turret;
    private Limelight3A limelight;

    // ================= TUNING CONSTANTS =================
    public double TURRET_kP = 0.019;
    public double TURRET_kD = 0.1;

    // ===== LIMELIGHT MICRO CORRECTION =====
    public double LL_MICRO_kP = 0.02;
    public double LL_MICRO_MAX = 0.4;

    // ===== SAFETY LIMITS =====
    public double TURRET_MAX_POWER = 0.8;
    public int TURRET_MIN_TICKS = -175;
    public int TURRET_MAX_TICKS = 175;

    // ===== GOAL COORDINATES =====
    public double TARGET_X = 14;      // Blue Goal X
    public double TARGET_Y = 131;     // Blue Goal Y

    // ================= STATE =================
    private double lastError = 0;
    private boolean usingLimelight = false;

    @Override
    public void init() {
        // Hardware Init
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(37, 72, Math.toRadians(90)));

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addLine("Ready to Tune.");
        telemetry.addLine("Press A for Pedro Tracking (Odometry)");
        telemetry.addLine("Press Y for Limelight Tracking (Vision)");
        telemetry.update();
    }

    // ================= THIS WAS MISSING =================
    @Override
    public void start() {
        follower.startTeleopDrive(); // <--- CRITICAL FIX
    }
    // ====================================================

    @Override
    public void loop() {
        follower.update();

        // Drive Control
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * 0.5,
                -gamepad1.left_stick_x * 0.5,
                -gamepad1.right_stick_x * 0.5,
                true
        );

        // Mode Switching
        if (gamepad1.a) usingLimelight = false;
        if (gamepad1.y) usingLimelight = true;

        double power = 0;
        int currentPos = turret.getCurrentPosition();
        double error = 0;
        String modeStatus = "";

        if (usingLimelight) {
            // === LIMELIGHT TUNING MODE ===
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                error = tx;

                power = tx * LL_MICRO_kP;
                power = Math.max(-LL_MICRO_MAX, Math.min(LL_MICRO_MAX, power));

                modeStatus = "LIMELIGHT (TX: " + String.format("%.2f", tx) + ")";
            } else {
                power = 0;
                modeStatus = "LIMELIGHT (NO TAG)";
            }
        } else {
            // === PEDRO TUNING MODE ===
            Pose pose = follower.getPose();

            // Check for null pose just in case, to prevent crash
            if (pose != null) {
                double botX = pose.getX();
                double botY = pose.getY();
                double botHeading = Math.toDegrees(pose.getHeading());

                double angleToGoal = Math.toDegrees(Math.atan2(TARGET_Y - botY, TARGET_X - botX));
                double targetTurretAngle = normalize(angleToGoal - botHeading);

                error = targetTurretAngle - currentPos;

                double derivative = error - lastError;
                power = (error * TURRET_kP) + (derivative * TURRET_kD);
                lastError = error;

                modeStatus = "PEDRO (Err: " + String.format("%.2f", error) + ")";
            } else {
                modeStatus = "WAITING FOR POSE...";
            }
        }

        // === GLOBAL SAFETY CLAMP ===
        power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));

        // === SOFT LIMITS (PROTECT CABLES) ===
        if ((power > 0 && currentPos >= TURRET_MAX_TICKS) ||
                (power < 0 && currentPos <= TURRET_MIN_TICKS)) {
            power = 0;
            modeStatus += " [LIMIT HIT]";
        }

        turret.setPower(power);

        // Telemetry
        telemetry.addData("Mode", modeStatus);
        telemetry.addData("Motor Power", power);
        telemetry.addData("Current Pos", currentPos);
        telemetry.update();
    }

    private double normalize(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }
}