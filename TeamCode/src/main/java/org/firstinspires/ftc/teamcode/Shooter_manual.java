package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter_Manual")
public class Shooter_manual extends OpMode {

    // ================= HARDWARE =================
    private DcMotor intake;
    private DcMotorEx shooterLeft, shooterRight;
    private Servo hood, leftRamp, rightRamp;

    // ================= CONSTANTS =================
    static final double TICKS_PER_REV = 28.0; // REV HD Hex

    // Shooter PIDF
    static final double kP = 60.0;
    static final double kI = 0.0;
    static final double kD = 6.0;
    static final double kF = 12.0;

    // Fixed shooting values
    static final double SHOOTER_RPM = 3900;
    static final double HOOD_POS    = 0.35;

    // Intake
    static final double INTAKE_POWER = -0.59;

    // Ramp positions
    static final double RAMP_UP   = 0.65;
    static final double RAMP_DOWN = 0.37;

    // ================= STATE =================
    private boolean shooterOn = false;
    private boolean intakeOn  = false;
    private boolean rampUp    = false;

    private boolean prevTriangle = false;
    private boolean prevX        = false;
    private boolean prevSquare   = false;

    // ================= INIT =================
    @Override
    public void init() {

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterRight = hardwareMap.get(DcMotorEx.class, "right_shooting");
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterLeft.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        shooterRight.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        hood = hardwareMap.get(Servo.class, "hood");
        leftRamp  = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");

        hood.setPosition(HOOD_POS);
        setRamp(false);
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        // -------- TOGGLE INPUTS --------
        if (gamepad1.triangle && !prevTriangle)
            shooterOn = !shooterOn;

        if (gamepad1.cross && !prevX)
            intakeOn = !intakeOn;

        if (gamepad1.square && !prevSquare)
            rampUp = !rampUp;

        prevTriangle = gamepad1.triangle;
        prevX        = gamepad1.cross;
        prevSquare   = gamepad1.square;

        // -------- SHOOTER --------
        if (shooterOn) {
            shooterLeft.setVelocity(rpmToTicks(SHOOTER_RPM));
            shooterRight.setVelocity(rpmToTicks(SHOOTER_RPM));
        } else {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }

        // -------- INTAKE --------
        intake.setPower(intakeOn ? INTAKE_POWER : 0);

        // -------- RAMP --------
        setRamp(rampUp);

        // -------- RPM TELEMETRY --------
        double leftRPM  =
                (shooterLeft.getVelocity() / TICKS_PER_REV) * 60.0;
        double rightRPM =
                (shooterRight.getVelocity() / TICKS_PER_REV) * 60.0;



        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Left RPM", "%.0f", leftRPM);
        telemetry.addData("Right RPM", "%.0f", rightRPM);
        telemetry.addData("Target RPM", SHOOTER_RPM);
        telemetry.addData("Hood Pos", HOOD_POS);
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Ramp", rampUp ? "UP" : "DOWN");
        telemetry.addData("Left Shooter Velocity", shooterLeft.getVelocity());

        telemetry.update();
    }

    // ================= HELPERS =================
    private double rpmToTicks(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    private void setRamp(boolean up) {
        double pos = up ? RAMP_UP : RAMP_DOWN;
        leftRamp.setPosition(pos);
        rightRamp.setPosition(1.0 - pos);
    }
}
