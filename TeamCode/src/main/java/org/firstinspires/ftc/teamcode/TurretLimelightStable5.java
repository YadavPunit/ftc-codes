package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@Configurable
@TeleOp(name = "Turret Limelight Stable5", group = "Test")
public class TurretLimelightStable5 extends OpMode {

    // ================= HARDWARE =================
    private DcMotorEx turret;
    private Limelight3A limelight;

    // ================= TUNING (Dashboard) =================

    // Proportional gain (vision → power)
    public static double kP = 0.012;

    // Maximum turret power
    public static double MAX_POWER = 0.30;

    // Deadband where turret starts moving
    public static double TX_DEADBAND = 1.2;

    // Inner deadband where turret holds still (hysteresis)
    public static double HOLD_DEADBAND = 0.3;

    // Minimum power to overcome gearbox stiction
    public static double MIN_POWER = 0.06;

    // Limelight tx filter strength (0–1)
    public static double TX_ALPHA = 0.7;

    // ================= INTERNAL =================
    private double filteredTx = 0;

    // ================= ENCODER + LIMITS =================
    private final double TICKS_PER_REV = 504.0;          // 28 × 18
    private final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;

    private final double MAX_ANGLE_DEG = 125.0;
    private final int MAX_TICKS = (int)(MAX_ANGLE_DEG * TICKS_PER_DEGREE);

    // ================= INIT =================
    @Override
    public void init() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(1);
        limelight.start();
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();
        int turretPos = turret.getCurrentPosition();
        double turretPower = 0;

        // -------- TARGET VISIBLE --------
        if (result != null && result.isValid()) {

            // Low-pass filter tx
            filteredTx = TX_ALPHA * filteredTx + (1 - TX_ALPHA) * result.getTx();
            double tx = filteredTx;

            // Outer deadband → move
            if (Math.abs(tx) > TX_DEADBAND) {

                turretPower = kP * tx;

                // Minimum power floor
                if (turretPower > 0) {
                    turretPower = Math.max(turretPower, MIN_POWER);
                } else {
                    turretPower = Math.min(turretPower, -MIN_POWER);
                }

            }
            // Inner deadband → hold still
            else if (Math.abs(tx) < HOLD_DEADBAND) {
                turretPower = 0;
            }
        }
        // -------- TARGET LOST --------
        else {
            filteredTx = 0;
            turretPower = 0;
        }

        // -------- SOFT LIMITS --------
        if (turretPos >= MAX_TICKS && turretPower > 0) {
            turretPower = 0;
        }
        if (turretPos <= -MAX_TICKS && turretPower < 0) {
            turretPower = 0;
        }

        // Clamp power
        turretPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, turretPower));

        turret.setPower(turretPower);

        // -------- TELEMETRY --------
        telemetry.addData("tx raw", result != null ? result.getTx() : "No Target");
        telemetry.addData("tx filtered", filteredTx);
        telemetry.addData("Turret Ticks", turretPos);
        telemetry.addData("Turret Degrees", turretPos / TICKS_PER_DEGREE);
        telemetry.addData("Turret Power", turretPower);
        telemetry.addData("Limits", "±" + MAX_TICKS);
        telemetry.update();
    }
}
