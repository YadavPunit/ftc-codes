package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LL_RAW_TEST")
public class LL_RAW_TEST extends OpMode {

    Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // DEFAULT PIPELINE
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        LLResult r = limelight.getLatestResult();

        telemetry.addLine("Loop running");

        if (r == null) {
            telemetry.addLine("Result = null");
        } else {
            telemetry.addData("Valid", r.isValid());
            telemetry.addData("Tx", r.getTx());
            telemetry.addData("Ty", r.getTy());
            telemetry.addData("Ta", r.getTa());
        }

        telemetry.update();
    }
}
