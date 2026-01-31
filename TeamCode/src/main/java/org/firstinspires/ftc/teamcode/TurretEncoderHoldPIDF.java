package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "TURRET_ENCODER_HOLD_PIDF", group = "Tuning")
public class TurretEncoderHoldPIDF extends LinearOpMode {

    /* =====================================================
       HARDWARE
       ===================================================== */
    private DcMotorEx turret;

    /* =====================================================
       TURRET TARGET (WRITE VALUE HERE)
       ===================================================== */
    private int turretTargetTicks = 0;   // 👈 WRITE TARGET HERE

    /* =====================================================
       PIDF CONSTANTS (TUNE THESE)
       ===================================================== */
    private static final double kP = 0.005;
    private static final double kI = 0.000021;
    private static final double kD = 0.006;
    private static final double kF = 0.06;

    /* =====================================================
       SAFETY
       ===================================================== */
    private static final double MAX_POWER = 0.8;
    private static final int DEAD_BAND_TICKS = 5;
    private static final int INTEGRAL_LIMIT = 300;

    private static final int TURRET_MIN_TICKS = -430;
    private static final int TURRET_MAX_TICKS = 430;

    /* =====================================================
       PID STATE
       ===================================================== */
    private double integral = 0;
    private double lastError = 0;

    /* =====================================================
       INIT
       ===================================================== */
    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("=== TURRET ENCODER HOLD PIDF ===");
        telemetry.addLine("Edit turretTargetTicks in code");
        telemetry.addLine("Turret will move and HOLD position");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        /* =====================================================
           SET TARGET (ONLY ONCE)
           ===================================================== */
        setTurretTargetTicks(320);   // 👈 CHANGE THIS VALUE

        /* =====================================================
           MAIN LOOP
           ===================================================== */
        while (opModeIsActive()) {
            updateTurretHoldPIDF();
            sendTelemetry();
        }
    }

    /* =====================================================
       SET TARGET FUNCTION (THIS IS WHERE YOU WRITE TARGET)
       ===================================================== */
    private void setTurretTargetTicks(int ticks) {
        turretTargetTicks = clampInt(ticks, TURRET_MIN_TICKS, TURRET_MAX_TICKS);

        // Reset PID memory
        integral = 0;
        lastError = 0;
    }

    /* =====================================================
       PIDF POSITION HOLD LOOP
       ===================================================== */
    private void updateTurretHoldPIDF() {

        int currentTicks = turret.getCurrentPosition();
        double error = turretTargetTicks - currentTicks;

        // Deadband → still HOLDING (PID still active)
        if (Math.abs(error) < DEAD_BAND_TICKS) {
            turret.setPower(0);
            integral = 0;
            lastError = error;
            return;
        }

        // Integral
        integral += error;
        integral = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        // Derivative
        double derivative = error - lastError;

        // PIDF output
        double power =
                (kP * error) +
                        (kI * integral) +
                        (kD * derivative) +
                        (kF * Math.signum(error));

        power = clamp(power, -MAX_POWER, MAX_POWER);

        // Safety limits
        if ((power > 0 && currentTicks >= TURRET_MAX_TICKS) ||
                (power < 0 && currentTicks <= TURRET_MIN_TICKS)) {
            power = 0;
        }

        turret.setPower(power);
        lastError = error;
    }

    /* =====================================================
       TELEMETRY
       ===================================================== */
    private void sendTelemetry() {

        telemetry.addLine("----- TURRET HOLD DATA -----");
        telemetry.addData("Target Ticks", turretTargetTicks);
        telemetry.addData("Current Ticks", turret.getCurrentPosition());
        telemetry.addData("Error", turretTargetTicks - turret.getCurrentPosition());
        telemetry.addData("Integral", integral);

        telemetry.addLine("PIDF");
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);

        telemetry.update();
    }

    /* =====================================================
       UTILITY
       ===================================================== */
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private int clampInt(int v, int min, int max) {
        return Math.max(min, Math.min(max, v));
    }
}