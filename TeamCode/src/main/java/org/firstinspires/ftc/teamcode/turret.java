package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret AprilTag Align")
public class turret extends OpMode {

    private Limelight3A limelight;
    private DcMotor turret;

    // Tuning values
    private static final double kP = 0.02;     // proportional gain
    private static final double MAX_POWER = 0.4;
    private static final double DEADZONE = 0.5; // degrees

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // AprilTag pipeline
        limelight.start();

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            // Limelight mounted vertically → tx/ty swapped
            double tx = result.getTx();
            double ty = result.getTy();

            // Use TY for turret yaw (vertical mount correction)
            double turretError = -ty;   // flip sign if needed

            double turretPower = 0.0;

            if (Math.abs(turretError) > DEADZONE) {
                turretPower = turretError * kP;
                turretPower = Math.max(-MAX_POWER,
                        Math.min(MAX_POWER, turretPower));
            }

            turret.setPower(turretPower);

            telemetry.addData("Turret Error", turretError);
            telemetry.addData("Turret Power", turretPower);
        } else {
            turret.setPower(0);
            telemetry.addLine("No AprilTag detected");
        }

        telemetry.update();
    }
}
