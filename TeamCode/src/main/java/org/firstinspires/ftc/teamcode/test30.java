package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "test30")
public class test30 extends OpMode {

    /* ================= CORE ================= */
    private Follower follower;
    private TelemetryManager telemetryM;

    /* ================= HARDWARE ================= */
    private DcMotor intake;
    private DcMotorEx shooterL, shooterR;
    private DcMotorEx turret;
    private Servo hood, leftRamp, rightRamp, leftLift, rightLift;
    private Limelight3A limelight;

    /* ================= CONSTANTS ================= */
    static final double TICKS_PER_REV = 28.0;
    static final double SLOW_MODE = 0.5;
    static final double INTAKE_POWER = -0.72;
    static final double RAMP_MIN_RPM = 2000;
    public static double TURN_SCALE = 0.6;

    /* ================= SHOOTER PIDF ================= */
    public static double SHOOTER_kP = 125;
    public static double SHOOTER_kI = 0;
    public static double SHOOTER_kD = 35;
    public static double SHOOTER_kF = 19;

    /* ================= PEDRO TURRET ================= */
    public static double TURRET_kP = 0.019;
    public static double TURRET_kD = 0.1;
    public static double TURRET_MAX_POWER = 0.8;
    public static double TURRET_SLEW = 1.0;
    public static double TURRET_TURN_COMP = 0;

    static final int TURRET_MIN_TICKS = -175;
    static final int TURRET_MAX_TICKS = 175;

    /* ================= LIMELIGHT PID + OFFSET (ROBOT Y BASED) ================= */

    // CLOSE RANGE
    public static double LL_CLOSE_kP = 0.018;
    public static double LL_CLOSE_OFFSET_kP = 0.0025;
    public static double LL_CLOSE_OFFSET_MAX = 4.0;

    // FAR RANGE
    public static double LL_FAR_kP = 0.008;
    public static double LL_FAR_OFFSET_kP = 0.0012;
    public static double LL_FAR_OFFSET_MAX = 2.0;

    public static double LL_Y_CLOSE_THRESHOLD = 49;
    public static double LL_DEADBAND = 0.3;

    /* ================= PRESETS ================= */
    static final double SHORT_RPM = 3000;
    static final double SHORT_HOOD = 0.34;
    static final double LONG_RPM = 4090;
    static final double LONG_HOOD = 0.28;

    /* ================= RAMP ================= */
    static final double RAMP_UP_LEFT = 0.55;
    static final double RAMP_UP_RIGHT = 0.45;
    static final double RAMP_DOWN_LEFT = 0.3;
    static final double RAMP_DOWN_RIGHT = 0.7;

    /* ================= LIFT ================= */
    static final double LIFT_UP_LEFT = 1.0;
    static final double LIFT_UP_RIGHT = 0.12;
    static final double LIFT_DOWN_LEFT = 0.2;
    static final double LIFT_DOWN_RIGHT = 0.8;

    /* ================= STATE ================= */
    private double lastTurretError = 0;
    private double lastTurretPower = 0;
    private double lastTurnInput = 0;
    private boolean intakeToggle = false;
    private boolean prevX = false;

    static final Pose START_POSE =
            new Pose(37.234, 71.776, Math.toRadians(90));

    /* ================= INIT ================= */
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterL = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterR = hardwareMap.get(DcMotorEx.class, "right_shooting");
        shooterR.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterR.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        leftRamp = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");
        leftLift = hardwareMap.get(Servo.class, "left_lift");
        rightLift = hardwareMap.get(Servo.class, "right_lift");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        setRamp(false);
        setLift(false);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /* ================= LOOP ================= */
    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        /* ===== DRIVE (UNCHANGED FROM test28) ===== */
        double speedMul = gamepad1.right_bumper ? SLOW_MODE : 0.5;
        lastTurnInput = -gamepad1.right_stick_x * TURN_SCALE * speedMul;
        if (Math.abs(lastTurnInput) < 0.02) lastTurnInput = 0;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedMul,
                -gamepad1.left_stick_x * speedMul,
                lastTurnInput,
                true
        );

        /* ===== TURRET ===== */
        LLResult ll = limelight.getLatestResult();
        boolean tagVisible = ll != null && ll.isValid();

        if (gamepad2.left_bumper && tagVisible) {
            alignTurretLimelight(ll);
        } else {
            alignTurretPedro();
        }

        /* ===== LIFT ===== */
        if (gamepad1.dpad_up) setLift(true);
        else if (gamepad1.dpad_down) setLift(false);

        /* ===== INTAKE (UNCHANGED) ===== */
        double intakePower = 0;
        boolean triggerActive = false;

        if (gamepad1.left_trigger > 0.05) {
            intakePower = -0.72 * gamepad1.left_trigger;
            triggerActive = true;
        } else if (gamepad1.right_trigger > 0.05) {
            intakePower = gamepad1.right_trigger;
            triggerActive = true;
        }

        if (triggerActive) intake.setPower(intakePower);
        else {
            if (gamepad2.cross && !prevX) intakeToggle = !intakeToggle;
            prevX = gamepad2.cross;
            intake.setPower(intakeToggle ? INTAKE_POWER : 0);
        }

        /* ===== SHOOTER ===== */
        if (gamepad2.left_bumper && tagVisible) {
            applyLimelightShootMap(ll);
        } else {
            if (gamepad2.triangle) {
                runShooterRPM(SHORT_RPM);
                hood.setPosition(SHORT_HOOD);
            } else if (gamepad2.circle) {
                runShooterRPM(LONG_RPM);
                hood.setPosition(LONG_HOOD);
            } else if (gamepad2.square) {
                shooterL.setVelocity(0);
                shooterR.setVelocity(0);
            }
        }

        /* ===== RAMP ===== */
        double rpm = Math.abs(shooterL.getVelocity()) / TICKS_PER_REV * 60.0;
        if (rpm < RAMP_MIN_RPM) {
            setRamp(false);
        } else {
            if (gamepad2.dpad_up) setRamp(true);
            else if (gamepad2.dpad_down) setRamp(false);
        }
    }

    /* ================= LIMELIGHT TURRET ================= */
    private void alignTurretLimelight(LLResult ll) {

        double ty = ll.getTy();
        if (Math.abs(ty) < LL_DEADBAND) {
            applyTurretPower(0);
            return;
        }

        double robotY = follower.getPose().getY();
        boolean close = robotY > LL_Y_CLOSE_THRESHOLD;

        double kP = close ? LL_CLOSE_kP : LL_FAR_kP;
        double oKP = close ? LL_CLOSE_OFFSET_kP : LL_FAR_OFFSET_kP;
        double oMax = close ? LL_CLOSE_OFFSET_MAX : LL_FAR_OFFSET_MAX;

        double pid = ty * kP;
        double offset = clamp(ty * oKP, -oMax, oMax);

        applyTurretPower(-(pid + offset));
    }

    /* ================= PEDRO TURRET ================= */
    private void alignTurretPedro() {

        Pose pose = follower.getPose();
        double fieldAngle = Math.toDegrees(
                Math.atan2(131 - pose.getY(), 14 - pose.getX())
        );

        double target = normalize(fieldAngle - Math.toDegrees(pose.getHeading()));
        double current = turret.getCurrentPosition();

        double error = target - current;
        double d = error - lastTurretError;
        lastTurretError = error;

        applyTurretPower(TURRET_kP * error + TURRET_kD * d);
    }

    /* ================= APPLY POWER ================= */
    private void applyTurretPower(double power) {

        power += lastTurnInput * TURRET_TURN_COMP;
        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        double delta = clamp(power - lastTurretPower, -TURRET_SLEW, TURRET_SLEW);
        power = lastTurretPower + delta;
        lastTurretPower = power;

        int pos = turret.getCurrentPosition();
        if ((power > 0 && pos >= TURRET_MAX_TICKS) ||
                (power < 0 && pos <= TURRET_MIN_TICKS)) {
            power = 0;
        }

        turret.setPower(power);
    }

    /* ================= LIMELIGHT SHOOT MAP ================= */
    private void applyLimelightShootMap(LLResult ll) {

        double tx = ll.getTx();
        double rpm, hoodPos;

        if (tx >= 15.77) { rpm = 3820; hoodPos = 0.25; }
        else if (tx >= 14.18) { rpm = map(tx,14.18,15.77,3360,3630); hoodPos = map(tx,14.18,15.77,0.28,0.25); }
        else if (tx >= 13.18) { rpm = map(tx,13.18,14.18,3200,3350); hoodPos = map(tx,13.18,14.18,0.39,0.28); }
        else if (tx >= 11.92) { rpm = map(tx,11.92,13.18,3050,3200); hoodPos = map(tx,11.92,13.18,0.41,0.39); }
        else if (tx >= 9.96) { rpm = map(tx,9.96,11.92,2900,3050); hoodPos = map(tx,9.96,11.92,0.45,0.41); }
        else if (tx >= 8.08) { rpm = map(tx,8.08,9.96,2800,2900); hoodPos = map(tx,8.08,9.96,0.49,0.45); }
        else if (tx >= 6.42) { rpm = map(tx,6.42,8.08,2660,2800); hoodPos = map(tx,6.42,8.08,0.52,0.49); }
        else if (tx >= 3.50) { rpm = map(tx,3.50,6.42,2600,2660); hoodPos = map(tx,3.50,6.42,0.60,0.52); }
        else if (tx >= 1.14) { rpm = map(tx,1.14,3.50,2540,2600); hoodPos = map(tx,1.14,3.50,0.75,0.60); }
        else { rpm = 2470; hoodPos = 0.89; }

        runShooterRPM(rpm);
        hood.setPosition(hoodPos);
    }

    /* ================= HELPERS ================= */
    private void runShooterRPM(double rpm) {
        double ticks = (rpm / 60.0) * TICKS_PER_REV;
        shooterL.setVelocity(ticks);
        shooterR.setVelocity(ticks);
    }

    private void setRamp(boolean up) {
        leftRamp.setPosition(up ? RAMP_UP_LEFT : RAMP_DOWN_LEFT);
        rightRamp.setPosition(up ? RAMP_UP_RIGHT : RAMP_DOWN_RIGHT);
    }

    private void setLift(boolean up) {
        leftLift.setPosition(up ? LIFT_UP_LEFT : LIFT_DOWN_LEFT);
        rightLift.setPosition(up ? LIFT_UP_RIGHT : LIFT_DOWN_RIGHT);
    }

    private double normalize(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);
    }
}
