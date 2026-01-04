package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "turret_align", group = "Test")
public class turret_align extends LinearOpMode {

    // ================= HARDWARE =================
    private DcMotorEx turret;
    private Limelight3A limelight;

    // ================= TURRET LIMITS =================
    static final int TURRET_MIN = -126;
    static final int TURRET_MAX =  126;

    // ================= SMOOTH PID =================
    static final double kP = 0.010;
    static final double kD = 0.0008;

    static final double ERROR_ALPHA = 0.2;
    static final double POWER_SLEW  = 0.04;
    static final double MAX_POWER  = 0.30;
    static final double DEADBAND   = 0.12;

    static final double TY_OFFSET = -8.0;

    // ================= STATE =================
    private double lastError = 0;
    private double filteredError = 0;
    private double lastPower = 0;

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.setPollRateHz(100);
        limelight.start();

        telemetry.addLine("Turret PID Auto Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult ll = limelight.getLatestResult();
            if (ll == null || !ll.isValid()) {
                turret.setPower(0);
                continue;
            }

            // -------- PID CALCULATION --------
            double rawError = -(ll.getTy() + TY_OFFSET);

            filteredError = (ERROR_ALPHA * rawError)
                    + ((1 - ERROR_ALPHA) * filteredError);

            double derivative = filteredError - lastError;
            lastError = filteredError;

            double output = (kP * filteredError) + (kD * derivative);

            // Deadband
            if (Math.abs(filteredError) < DEADBAND) {
                output = 0;
            }

            // Clamp
            output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));

            // Slew-rate limit
            double delta = output - lastPower;
            delta = Math.max(-POWER_SLEW, Math.min(POWER_SLEW, delta));
            output = lastPower + delta;
            lastPower = output;

            // Encoder soft limits
            int pos = turret.getCurrentPosition();
            if ((output < 0 && pos <= TURRET_MIN) ||
                    (output > 0 && pos >= TURRET_MAX)) {
                output = 0;
            }

            turret.setPower(output);

            telemetry.addData("TY", ll.getTy());
            telemetry.addData("FilteredErr", filteredError);
            telemetry.addData("TurretEnc", pos);
            telemetry.addData("Power", output);
            telemetry.update();
        }
    }
}
