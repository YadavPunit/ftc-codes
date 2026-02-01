package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@TeleOp(name = "TurretTuning_limelight")
public class TurretTuning_limelight extends OpMode {

    /* ================= HARDWARE ================= */
    private DcMotorEx turret;

    /* ================= CONSTANTS ================= */
    static final int TURRET_MIN_TICKS = -175;
    static final int TURRET_MAX_TICKS = 175;

    /* ================= TUNABLES ================= */
    public static double TURRET_kP = 0.027;
    public static double TURRET_kD = 0.16;
    public static double TURRET_MAX_POWER = 0.6;
    public static double TURRET_MANUAL_POWER = 0.5;
    public static double TURRET_SLEW = 0.75;

    /* ================= STATE ================= */
    private boolean autoMode = false;
    private double targetTicks = 0;

    private double lastError = 0;
    private double lastPower = 0;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        /* ================= MODE CONTROL ================= */
        if (gamepad1.cross) autoMode = true;
        if (gamepad1.triangle) autoMode = false;

        if (gamepad1.right_bumper) {
            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            targetTicks = 0;
        }

        /* ================= TARGET ADJUST ================= */
        if (gamepad1.dpad_right) targetTicks += 2;
        if (gamepad1.dpad_left)  targetTicks -= 2;

        targetTicks = clamp(targetTicks, TURRET_MIN_TICKS, TURRET_MAX_TICKS);

        double power = 0;

        /* ================= MANUAL MODE ================= */
        if (!autoMode) {
            double manual = -gamepad1.left_stick_x;

            if (Math.abs(manual) > 0.05) {
                power = manual * TURRET_MANUAL_POWER;
                targetTicks = turret.getCurrentPosition();
                lastError = 0;
            } else {
                power = 0;
            }
        }

        /* ================= AUTO PID MODE ================= */
        else {
            double current = turret.getCurrentPosition();
            double error = targetTicks - current;
            double derivative = error - lastError;
            lastError = error;

            power = (TURRET_kP * error) + (TURRET_kD * derivative);
        }

        /* ================= SLEW RATE ================= */
        double delta = clamp(power - lastPower, -TURRET_SLEW, TURRET_SLEW);
        power = lastPower + delta;
        lastPower = power;

        /* ================= CLAMP ================= */
        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        /* ================= SOFT LIMITS ================= */
        int pos = turret.getCurrentPosition();
        if ((power > 0 && pos >= TURRET_MAX_TICKS) ||
                (power < 0 && pos <= TURRET_MIN_TICKS)) {
            power = 0;
        }

        turret.setPower(power);

        /* ================= TELEMETRY ================= */
        telemetry.addLine(autoMode ? "MODE: AUTO" : "MODE: MANUAL");
        telemetry.addData("Target", targetTicks);
        telemetry.addData("Position", pos);
        telemetry.addData("Error", targetTicks - pos);
        telemetry.addData("Power", power);
        telemetry.addData("kP", TURRET_kP);
        telemetry.addData("kD", TURRET_kD);
        telemetry.update();
    }

    /* ================= HELPERS ================= */
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
