package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TUNE_TURRET_PIDF_APRILTAG", group = "Tuning")
public class TuneTurretPIDFAprilTag extends LinearOpMode {

    /* ================= HARDWARE ================= */
    private DcMotorEx turret;
    private Limelight3A limelight;

    /* ================= PIDF (START VALUES) ================= */
    // 👇 Start LOW — increase slowly
    private double kP = 0.005;
    private double kI = 0.0;
    private double kD = 0.002;
    private double kF = 0.06;

    /* ================= TURRET LIMITS ================= */
    private static final int TURRET_MIN_TICKS = -430;
    private static final int TURRET_MAX_TICKS = 430;
    private static final double TICKS_PER_DEGREE = 3.36;

    /* ================= PID STATE ================= */
    private int turretTargetTicks = 0;
    private double integral = 0;
    private double lastError = 0;

    private static final int DEAD_BAND_TICKS = 4;
    private static final int INTEGRAL_LIMIT = 250;
    private static final double MAX_POWER = 0.7;

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        limelight.start(); // make sure AprilTag pipeline is selected

        telemetry.addLine("TURRET APRILTAG PIDF TUNER READY");
        telemetry.addLine("Adjust PIDF in code, rerun OpMode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            updateTargetFromAprilTag();
            updateTurretPIDF();

            telemetry.addData("Tag Target Ticks", turretTargetTicks);
            telemetry.addData("Current Ticks", turret.getCurrentPosition());
            telemetry.addData("Error", turretTargetTicks - turret.getCurrentPosition());
            telemetry.addData("Integral", integral);

            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);

            telemetry.update();
        }
    }

    /* ================= APRILTAG → TARGET ================= */
    private void updateTargetFromAprilTag() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        // AprilTag horizontal yaw (degrees)
        double yaw = result.getTx(); // use getYaw() if your API provides it

        // Deadband to avoid jitter
        if (Math.abs(yaw) < 0.5) return;

        int offsetTicks = (int) (yaw * TICKS_PER_DEGREE);

        // Flip sign if turret moves wrong direction
        // offsetTicks *= -1;

        turretTargetTicks = clamp(
                turretTargetTicks + offsetTicks,
                TURRET_MIN_TICKS,
                TURRET_MAX_TICKS
        );
    }

    /* ================= PIDF CONTROL ================= */
    private void updateTurretPIDF() {

        int current = turret.getCurrentPosition();
        double error = turretTargetTicks - current;

        if (Math.abs(error) < DEAD_BAND_TICKS) {
            turret.setPower(0);
            integral = 0;
            lastError = error;
            return;
        }

        integral += error;
        integral = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        double derivative = error - lastError;

        double power =
                (kP * error) +
                        (kI * integral) +
                        (kD * derivative) +
                        (kF * Math.signum(error));

        power = clamp(power, -MAX_POWER, MAX_POWER);

        turret.setPower(-power);
        lastError = error;
    }

    /* ================= UTIL ================= */
    private int clamp(int v, int min, int max) {
        return Math.max(min, Math.min(max, v));
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
