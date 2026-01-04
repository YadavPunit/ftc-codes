package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter_Encoder_Telemetry_RunWithEncoder")
public class telemeatry_data extends OpMode {

    // ===== Motors =====
    private DcMotorEx shooting;
    private DcMotorEx shooting1;

    @Override
    public void init() {

        shooting  = hardwareMap.get(DcMotorEx.class, "shooting");
        shooting1 = hardwareMap.get(DcMotorEx.class, "shooting1");

        // Same direction as your robot
        shooting.setDirection(DcMotor.Direction.REVERSE);

        // 🔑 IMPORTANT: RUN USING ENCODER
        shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooting.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooting1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooting.setPower(0);
        shooting1.setPower(0);

        telemetry.addLine("Shooter Encoder Telemetry (RUN_USING_ENCODER)");
        telemetry.addLine("Press A = RUN | B = STOP");
        telemetry.update();
    }

    @Override
    public void loop() {

        // OPTIONAL motor spin for testing
        if (gamepad1.a) {
            shooting.setPower(0.5);
            shooting1.setPower(0.5);
        }

        if (gamepad1.b) {
            shooting.setPower(0);
            shooting1.setPower(0);
        }

        // ===== Encoder Velocity Telemetry =====
        double rpmShooting  = shooting.getVelocity();
        double rpmShooting1 = shooting1.getVelocity();

        telemetry.addData("Shooting RPM", rpmShooting);
        telemetry.addData("Shooting1 RPM", rpmShooting1);
        telemetry.update();
    }
}
