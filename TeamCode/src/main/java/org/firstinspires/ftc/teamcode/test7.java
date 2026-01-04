package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "test7")
public class test7 extends OpMode {

    // ================= HARDWARE =================
    private Follower follower;

    private DcMotor intake;
    private DcMotorEx shooting, shooting1;
    private DcMotorEx turret;

    private Servo leftRamp, rightRamp, hood;
    private Limelight3A limelight;

    // ================= CONSTANTS =================
    static final double TICKS_PER_REV = 28.0;

    // -------- Shooter Velocity PIDF (REV Hub) --------
    static final double SHOOTER_kP = 60.0;
    static final double SHOOTER_kI = 0.0;
    static final double SHOOTER_kD = 6.0;
    static final double SHOOTER_kF = 12.0;

    // -------- Turret limits (±90°) --------
    static final int TURRET_MIN = -126;
    static final int TURRET_MAX =  126;

    // -------- Turret PID (SMOOTH, NO WOBBLE) --------
    static final double TURRET_KP = 0.010;
    static final double TURRET_KD = 0.0008;

    static final double ERROR_ALPHA = 0.2;
    static final double POWER_SLEW  = 0.04;
    static final double TURRET_MAX_POWER = 0.30;
    static final double TURRET_DEADBAND  = 0.12;

    // Limelight mounting offset
    static final double TY_OFFSET = -8.0;

    // Servo limits
    static final double RAMP_MIN = 0.35;
    static final double RAMP_MAX = 0.70;
    static final double HOOD_MIN = 0.30;
    static final double HOOD_MAX = 1.00;

    // Intake
    static final double INTAKE_SHOOT = -0.4;
    static final double INTAKE_FULL  = -0.8;

    // ================= STATE =================
    private double turretLastError = 0;
    private double filteredError  = 0;
    private double lastTurretPower = 0;

    private boolean intakeToggle  = false;
    private boolean shooterToggle = false;
    private boolean rampToggle    = false;

    private boolean prevA = false, prevB = false, prevX = false;

    // ================= INIT =================
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        intake    = hardwareMap.get(DcMotor.class, "intake");
        shooting  = hardwareMap.get(DcMotorEx.class, "shooting");
        shooting1 = hardwareMap.get(DcMotorEx.class, "shooting1");
        turret    = hardwareMap.get(DcMotorEx.class, "turret");

        leftRamp  = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");
        hood      = hardwareMap.get(Servo.class, "hood");

        shooting.setDirection(DcMotor.Direction.REVERSE);
        shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // -------- APPLY SHOOTER PIDF --------
        shooting.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF
        );
        shooting1.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF
        );

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setRampPosition(0.55);
        setHoodPosition(0.50);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        // -------- DRIVE INPUT --------
        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double turn    =  gamepad1.right_stick_x;

        double speedScale = gamepad1.right_bumper ? 0.4 : 1.0;

        LLResult ll = limelight.getLatestResult();
        boolean hasTarget = ll != null && ll.isValid();

        // -------- TURRET ALIGNMENT (HOLD LB) --------
        if (gamepad1.left_bumper && hasTarget) {

            double rawError = -(ll.getTy() + TY_OFFSET);

            filteredError = (ERROR_ALPHA * rawError)
                    + ((1 - ERROR_ALPHA) * filteredError);

            double derivative = filteredError - turretLastError;
            turretLastError = filteredError;

            double output =
                    (TURRET_KP * filteredError) +
                            (TURRET_KD * derivative);

            if (Math.abs(filteredError) < TURRET_DEADBAND) {
                output = 0;
            }
            output = Math.max(-TURRET_MAX_POWER,
                    Math.min(TURRET_MAX_POWER, output));

            double delta = output - lastTurretPower;
            delta = Math.max(-POWER_SLEW, Math.min(POWER_SLEW, delta));
            output = lastTurretPower + delta;
            lastTurretPower = output;

            int pos = turret.getCurrentPosition();
            if ((output < 0 && pos <= TURRET_MIN) ||
                    (output > 0 && pos >= TURRET_MAX)) {
                output = 0;
            }

            turret.setPower(output);

            updateShooterAndHood(ll.getTx());
            intake.setPower(INTAKE_SHOOT);

        } else {
            turret.setPower(0);
            turretLastError = 0;
            filteredError = 0;
            lastTurretPower = 0;
        }

        // -------- DRIVE --------
        follower.setTeleOpDrive(
                forward * speedScale,
                strafe  * speedScale,
                turn    * speedScale,
                true
        );

        follower.update();

        // -------- GAMEPAD 2 TOGGLES --------
        if (gamepad2.a && !prevA) intakeToggle = !intakeToggle;
        if (gamepad2.b && !prevB) shooterToggle = !shooterToggle;
        if (gamepad2.x && !prevX) rampToggle = !rampToggle;

        prevA = gamepad2.a;
        prevB = gamepad2.b;
        prevX = gamepad2.x;

        if (!gamepad1.left_bumper) {
            intake.setPower(intakeToggle ? INTAKE_FULL : 0);
        }

        if (!shooterToggle && !gamepad1.left_bumper) {
            shooting.setPower(0);
            shooting1.setPower(0);
        }

        setRampPosition(rampToggle ? 0.65 : 0.40);

        // -------- TELEMETRY --------
        telemetry.addData("Turret Enc", turret.getCurrentPosition());
        telemetry.addData("Shooter RPM",
                (shooting.getVelocity() / TICKS_PER_REV) * 60.0);
        telemetry.update();
    }

    // ================= HELPERS =================
    private double map(double x, double inMin, double inMax,
                       double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin)
                / (inMax - inMin) + outMin;
    }

    private double rpmToTicks(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    private void setRampPosition(double pos) {
        pos = Math.max(RAMP_MIN, Math.min(RAMP_MAX, pos));
        leftRamp.setPosition(pos);
        rightRamp.setPosition(1.0 - pos);
    }

    private void setHoodPosition(double pos) {
        pos = Math.max(HOOD_MIN, Math.min(HOOD_MAX, pos));
        hood.setPosition(pos);
    }

    private void updateShooterAndHood(double aim) {

        aim = Math.max(4.14, Math.min(18.44, aim));

        double rpm, hoodPos;

        if (aim <= 9.42) {
            rpm = map(aim, 4.14, 9.42, 3800, 4100);
            hoodPos = map(aim, 4.14, 9.42, 0.50, 0.45);
        } else if (aim <= 15.79) {
            rpm = map(aim, 9.42, 15.79, 4100, 4600);
            hoodPos = map(aim, 9.42, 15.79, 0.45, 0.35);
        } else {
            rpm = map(aim, 15.79, 18.44, 4600, 5200);
            hoodPos = map(aim, 15.79, 18.44, 0.35, 0.30);
        }

        rpm = Math.max(3500, Math.min(5400, rpm));
        hoodPos = Math.max(HOOD_MIN, Math.min(HOOD_MAX, hoodPos));

        shooting.setVelocity(rpmToTicks(rpm));
        shooting1.setVelocity(rpmToTicks(rpm));
        setHoodPosition(hoodPos);
    }
}
