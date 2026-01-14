package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "turret_test5")
public class turret_test5 extends OpMode {

    // ================= HARDWARE =================
    private DcMotorEx turret;
    private Limelight3A limelight;

    // ================= ENCODER LIMITS =================
    // HD Hex (28 ticks) × 5:1 × 3.6 = 504 ticks / rev
    // 90° = 504 / 4 = 126 ticks
    static final int TURRET_MIN_TICKS = -126;
    static final int TURRET_MAX_TICKS =  126;

    // ================= PID =================
    static final double kP = 0.015;
    static final double kD = 0.0008;

    // ================= SMOOTHING =================
    static final double DEADBAND = 0.25;       // ty dead zone
    static final double MAX_POWER = 0.30;      // safe power
    static final double POWER_SLEW = 0.04;     // smooth accel
    static final double ERROR_ALPHA = 0.20;    // low-pass filter

    // ================= CAMERA OFFSET =================
    static final double TY_OFFSET = 0.0;       // tune if camera tilted

    // ================= STATE =================
    private double filteredError = 0;
    private double lastError = 0;
    private double lastPower = 0;

    // ================= INIT =================
    @Override
    public void init() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotor.Direction.REVERSE);

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

            // ===== ERROR (TY BASED) =====
            // Positive ty → positive power
            double rawError = ll.getTy() - TY_OFFSET;

            // ===== LOW-PASS FILTER =====
            filteredError =
                    ERROR_ALPHA * rawError +
                            (1.0 - ERROR_ALPHA) * filteredError;

            // ===== DEADBAND =====
            if (Math.abs(filteredError) < DEADBAND) {
                stopTurret();
                return;
            }

            // ===== DERIVATIVE =====
            double derivative = filteredError - lastError;
            lastError = filteredError;

            // ===== PD OUTPUT =====
            double power =
                    (kP * filteredError) +
                            (kD * derivative);

            power = clamp(power, -MAX_POWER, MAX_POWER);

            // ===== SLEW RATE LIMIT =====
            double delta = power - lastPower;
            delta = clamp(delta, -POWER_SLEW, POWER_SLEW);
            power = lastPower + delta;
            lastPower = power;

            // ===== HARD ENCODER LIMITS =====
            int pos = turret.getCurrentPosition();
            if ((power > 0 && pos >= TURRET_MAX_TICKS) ||
                    (power < 0 && pos <= TURRET_MIN_TICKS)) {
                stopTurret();
                return;
            }

            turret.setPower(power);

        } else {
            stopTurret();
        }

        // ===== TELEMETRY =====
        telemetry.addData("Turret Encoder", turret.getCurrentPosition());
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
