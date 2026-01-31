package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret_Limelight_Tuning", group = "Tuning")
public class Turret_Limelight_Tuning extends LinearOpMode {

    // ================= HARDWARE =================
    private DcMotorEx turret;
    private Limelight3A limelight;

    // ================= TURRET LIMITS =================
    static final int TURRET_MIN_TICKS = -175;
    static final int TURRET_MAX_TICKS =  175;

    // ================= TUNING PARAMS =================
    public static double TURRET_KP = 0.035;
    public static double TURRET_MAX_POWER = 0.6;
    public static double TURRET_DEADBAND = 0.9;

    // Change this live for shot-by-shot offset testing
    public static double tyOffset = 0.0;

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.start();

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0);

        telemetry.addLine("Turret Limelight Tuning Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                turret.setPower(0);
                telemetry.addLine("No Limelight Target");
                telemetry.update();
                continue;
            }

            // ===== SAME LOGIC AS AUTO =====
            double ty = result.getTy() + tyOffset;

            double power = 0;

            if (Math.abs(ty) > TURRET_DEADBAND) {
                power = ty * TURRET_KP;
                power = Math.max(-TURRET_MAX_POWER,
                        Math.min(TURRET_MAX_POWER, power));
            }

            int pos = turret.getCurrentPosition();

            // Enforce hard limits
            if ((pos <= TURRET_MIN_TICKS && power < 0) ||
                    (pos >= TURRET_MAX_TICKS && power > 0)) {
                power = 0;
            }

            turret.setPower(power);

            // ================= TELEMETRY =================
            telemetry.addData("TY Raw", result.getTy());
            telemetry.addData("TY Offset", tyOffset);
            telemetry.addData("TY Used", ty);
            telemetry.addData("Turret Pos", pos);
            telemetry.addData("Power", power);
            telemetry.addData("KP", TURRET_KP);
            telemetry.addData("Deadband", TURRET_DEADBAND);
            telemetry.update();
        }

        turret.setPower(0);
    }
}
