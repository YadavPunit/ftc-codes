package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@TeleOp(name = "turret_final_tuner1", group = "Test")
public class turret_final_tuner1 extends OpMode {

    /* ================= HARDWARE ================= */
    private DcMotorEx turret;
    private Limelight3A limelight;

    /* ================= TURRET CONSTANTS ================= */

    // PD (FINAL TUNED VALUES)
    public static double TURRET_kP = 0.0028;
    public static double TURRET_kD = 0.05;
    public static double TURRET_kF = 0.17;

    public static double TURRET_MAX_POWER = 0.8;
    public static double TURRET_SLEW = 0.04;

    static final int TURRET_MIN_TICKS = -250;
    static final int TURRET_MAX_TICKS =  250;

    static final double TY_DEADBAND = 0.15;

    /* ================= STATE ================= */
    private double lastError = 0;
    private double lastPower = 0;

    /* ================= INIT ================= */
    @Override
    public void init() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    /* ================= LOOP ================= */
    @Override
    public void loop() {

        if (gamepad2.left_bumper) {
            alignTurretToTy();
        } else {
            turret.setPower(0);
            lastError = 0;
            lastPower = 0;
        }

        sendTelemetry();
    }

    /* ================= LIMELIGHT TY PD ================= */
    private void alignTurretToTy() {

        LLResult ll = limelight.getLatestResult();
        if (ll == null || !ll.isValid()) {
            turret.setPower(0);
            return;
        }

        // We want TY → 0
        double error = ll.getTy();

        if (Math.abs(error) < TY_DEADBAND) {
            turret.setPower(0);
            lastError = 0;
            lastPower = 0;
            return;
        }

        double derivative = error - lastError;

        double power = (TURRET_kP * error)
                + (TURRET_kD * derivative)
                + Math.signum(error) * TURRET_kF;

        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        // Slew rate limiting (EXACT from your tuner)
        double delta = clamp(power - lastPower, -TURRET_SLEW, TURRET_SLEW);
        power = lastPower + delta;

        int pos = turret.getCurrentPosition();
        if ((power > 0 && pos >= TURRET_MAX_TICKS) ||
                (power < 0 && pos <= TURRET_MIN_TICKS)) {
            power = 0;
        }

        turret.setPower(power);

        lastError = error;
        lastPower = power;
    }

    /* ================= TELEMETRY ================= */
    private void sendTelemetry() {

        telemetry.addLine("==== LIMELIGHT TY TUNER ====");
        telemetry.addData("TY",
                limelight.getLatestResult() != null ?
                        limelight.getLatestResult().getTy() : "no target");
        telemetry.addData("Turret Ticks", turret.getCurrentPosition());
        telemetry.addData("Power", lastPower);

        telemetry.addLine("==== PIDF ====");
        telemetry.addData("kP", TURRET_kP);
        telemetry.addData("kD", TURRET_kD);
        telemetry.addData("kF", TURRET_kF);

        telemetry.update();
    }

    /* ================= UTILS ================= */
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
