package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "turret_test3")
public class turret_test3 extends OpMode {

    // ================= HARDWARE =================
    private DcMotorEx turret;
    private Limelight3A limelight;

    // ================= PID CONSTANTS =================
    static final double kP = 0.025;   // proportional gain
    static final double kD = 0.002;   // derivative gain

    static final double DEADBAND = 0.25;  // ty dead zone
    static final double MAX_POWER = 0.4;  // safety limit

    // ================= STATE =================
    private double lastError = 0;

    @Override
    public void init() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    @Override
    public void loop() {

        double power = 0;

        LLResult ll = limelight.getLatestResult();
        boolean hasTarget = ll != null && ll.isValid();

        if (hasTarget) {

            // 🔥 READ TY
            double ty = ll.getTy();   // vertical offset

            // 🔥 DEAD BAND (NO JITTER)
            if (Math.abs(ty) < DEADBAND) {
                power = 0;
                lastError = 0;
            } else {

                // 🔥 PID (PD CONTROL)
                double error = ty;                 // sign preserved
                double derivative = error - lastError;
                lastError = error;

                power = (kP * error) + (kD * derivative);
            }
        } else {
            power = 0;
            lastError = 0;
        }

        // 🔒 SAFETY CLAMP
        power = clamp(power, -MAX_POWER, MAX_POWER);

        turret.setPower(power);

        telemetry.addData("Has Target", hasTarget);
        telemetry.addData("ty", hasTarget ? ll.getTy() : 0);
        telemetry.addData("Turret Power", power);
        telemetry.update();
    }

    // ================= HELPER =================
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
