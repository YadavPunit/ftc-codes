package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter_Velocity_Mapped_Test")
public class ShooterVelocityPIDTest extends OpMode {

    // ===== Motors =====
    private DcMotorEx shooting;
    private DcMotorEx shooting1;

    // ===== Velocity =====
    private static final double MAX_ENCODER_RPM = 2300.0; // OBSERVED MAX
    private double targetRPM = 0;

    private static final double RPM_STEP = 100;

    @Override
    public void init() {

        shooting  = hardwareMap.get(DcMotorEx.class, "shooting");
        shooting1 = hardwareMap.get(DcMotorEx.class, "shooting1");

        shooting.setDirection(DcMotor.Direction.REVERSE);

        // 🔑 REQUIRED for setVelocity
        shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooting.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooting1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooting.setVelocity(0);
        shooting1.setVelocity(0);

        telemetry.addLine("Shooter Velocity Mapped Test READY");
        telemetry.addLine("DPAD UP/DOWN → Change RPM");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ===== DPAD CONTROL =====
        if (gamepad1.dpad_up)   targetRPM += RPM_STEP;
        if (gamepad1.dpad_down) targetRPM -= RPM_STEP;

        targetRPM = clip(targetRPM, 0, MAX_ENCODER_RPM);

        // ===== APPLY VELOCITY =====
        shooting.setVelocity(targetRPM);
        shooting1.setVelocity(targetRPM);

        // ===== READ ENCODER VELOCITY =====
        double rpmLeft  = shooting.getVelocity();
        double rpmRight = shooting1.getVelocity();

        // ===== MAP TO % =====
        double rpmPercentLeft  = map(rpmLeft,  0, MAX_ENCODER_RPM, 0, 100);
        double rpmPercentRight = map(rpmRight, 0, MAX_ENCODER_RPM, 0, 100);

        // ===== TELEMETRY =====
        telemetry.addData("Target RPM (encoder)", targetRPM);
        telemetry.addData("Shooter RPM Left", rpmLeft);
        telemetry.addData("Shooter RPM Right", rpmRight);
        telemetry.addData("Shooter % Left", "%.1f %%", rpmPercentLeft);
        telemetry.addData("Shooter % Right", "%.1f %%", rpmPercentRight);
        telemetry.update();
    }

    // ===== UTILITIES =====
    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double map(double x, double inMin, double inMax,
                       double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin)
                / (inMax - inMin) + outMin;
    }
}