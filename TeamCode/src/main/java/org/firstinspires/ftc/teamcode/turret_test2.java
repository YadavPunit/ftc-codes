package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "turret_test2")
public class turret_test2 extends OpMode {

    private DcMotorEx turret;
    private Limelight3A limelight;

    // SIMPLE gain (no PID yet)
    static final double kP = 0.02;

    @Override
    public void init() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    @Override
    public void loop() {

        boolean align = gamepad1.left_bumper;

        LLResult ll = limelight.getLatestResult();
        boolean hasTarget = ll != null && ll.isValid();

        double power = 0;

        if (align && hasTarget) {
            double error = ll.getTx();   // ✅ CORRECT AXIS
            power = kP * error;
        }

        power = clamp(power, -0.4, 0.4);
        turret.setPower(power);

        telemetry.addData("Align", align);
        telemetry.addData("Has Target", hasTarget);
        telemetry.addData("ty", hasTarget ? ll.getTx() : 0);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
