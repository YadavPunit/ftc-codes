package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "turret_test")
public class turret_test extends OpMode {

    // ================= HARDWARE =================
    private DcMotorEx turret;
    private Limelight3A limelight;

    // ================= TURRET LIMITS =================
    static final int TURRET_MIN_TICKS = -126;
    static final int TURRET_MAX_TICKS =  126;

    // ================= PID =================
    static final double kP = 0.015;
    static final double kD = 0.0009;

    // ================= SMOOTHING =================
    static final double ERROR_FILTER_ALPHA = 0.20;
    static final double POWER_SLEW = 0.04;
    static final double MAX_POWER = 0.30;
    static final double DEADBAND = 0.15;

    // ================= TY ALIGN CONFIG =================
    static final double TY_ALIGN_OFFSET = 0.0;

    // ================= STATE =================
    private double filteredError = 0;
    private double lastError = 0;
    private double lastPower = 0;

    // ================= INIT =================
    @Override
    public void init() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        // 🔥 FINAL, CORRECT FIX
        turret.setDirection(DcMotorEx.Direction.REVERSE);

        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        boolean align = gamepad1.left_bumper;

        LLResult ll = limelight.getLatestResult();
        boolean hasTarget = ll != null && ll.isValid();

        if (align && hasTarget) {

            // ===== TY ERROR =====
            double rawError = ll.getTy() - TY_ALIGN_OFFSET;

            // ===== FILTER =====
            filteredError =
                    ERROR_FILTER_ALPHA * rawError +
                            (1.0 - ERROR_FILTER_ALPHA) * filteredError;

            // ===== DERIVATIVE =====
            double derivative = filteredError - lastError;
            lastError = filteredError;

            // ===== PD =====
            double output =
                    (kP * filteredError) +
                            (kD * derivative);

            // ===== SOFT DEADBAND =====
            if (Math.abs(filteredError) < DEADBAND) {
                output = 0;
            }

            // ===== CLAMP =====
            output = clamp(output, -MAX_POWER, MAX_POWER);

            // ===== SLEW =====
            double delta = output - lastPower;
            delta = clamp(delta, -POWER_SLEW, POWER_SLEW);
            output = lastPower + delta;
            lastPower = output;

            // ===== LIMITS =====
            int pos = turret.getCurrentPosition();
            if ((output < 0 && pos <= TURRET_MIN_TICKS) ||
                    (output > 0 && pos >= TURRET_MAX_TICKS)) {
                output = 0;
            }

            turret.setPower(output);

        } else {
            stopTurret();
        }

        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.addData("ty", hasTarget ? ll.getTy() : 0);
        telemetry.addData("Filtered Error", filteredError);
        telemetry.addData("Power", lastPower);
        telemetry.update();
    }

    // ================= HELPERS =================
    private void stopTurret() {
        turret.setPower(0);
        lastPower = 0;
        lastError = 0;
        filteredError = 0;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
