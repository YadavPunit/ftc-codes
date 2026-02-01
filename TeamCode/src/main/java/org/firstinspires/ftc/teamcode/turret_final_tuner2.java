package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "turret_final_tuner2", group = "Test")
public class turret_final_tuner2 extends OpMode {

    /* ================= PEDRO ================= */
    private Follower follower;

    /* ================= HARDWARE ================= */
    private DcMotorEx turret;

    /* ================= DRIVE ================= */
    static final double SLOW_MODE = 0.5;
    static final double TURN_SCALE = 0.6;

    /* ================= ALLIANCE GOAL (APRILTAG) ================= */
    public static double GOAL_X = 14;
    public static double GOAL_Y = 131;

    /* ================= TURRET CONSTANTS ================= */
    static final double TICKS_PER_DEGREE = 1.4;

    static final int TURRET_MIN_TICKS = -175;
    static final int TURRET_MAX_TICKS = 175;

    public static double TURRET_kP = 0.0025;
    public static double TURRET_kD = 0.055;
    public static double TURRET_kF = 0.17;

    public final double TURRET_MAX_POWER = 0.8;
    public final double TURRET_SLEW = 0.04;

    /* ================= STATE ================= */
    private double targetDeg = 0;
    private int targetTicks = 0;

    private double lastError = 0;
    private double lastPower = 0;

    /* Passive alignment */
    public static boolean PASSIVE_GOAL_ALIGN = true;
    private boolean latchedInRange = false;

    /* ================= INIT ================= */
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(37.234, 71.776, Math.toRadians(90)));
        follower.startTeleopDrive();

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /* ================= LOOP ================= */
    @Override
    public void loop() {

        driveRobot();

        if (PASSIVE_GOAL_ALIGN) {
            passiveGoalAlignment();
        } else {
            handleTurretInputs();
        }

        updateTurretPID();
        follower.update();
        sendTelemetry();
    }

    /* ================= DRIVE ================= */
    private void driveRobot() {

        double speedMul = gamepad1.right_bumper ? SLOW_MODE : 1.0;

        double forward = -gamepad1.left_stick_y * speedMul;
        double strafe  = -gamepad1.left_stick_x * speedMul;
        double turn    = -gamepad1.right_stick_x * TURN_SCALE * speedMul;

        if (Math.abs(turn) < 0.02) turn = 0;

        follower.setTeleOpDrive(forward, strafe, turn, true);
    }

    /* ================= PASSIVE GOAL ALIGN ================= */
    private void passiveGoalAlignment() {

        Pose pose = follower.getPose();

        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();

        /* Angle from robot to goal in field frame */
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        /* Robot heading (0 right, 90 front) */
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        /* Turret relative angle (0 = robot front) */
        double turretDeg = normalizeDeg(fieldAngleDeg + robotHeadingDeg + 90);

        int desiredTicks = (int) (turretDeg * TICKS_PER_DEGREE);

        boolean inRange =
                desiredTicks >= TURRET_MIN_TICKS &&
                        desiredTicks <= TURRET_MAX_TICKS;

        if (inRange) {
            targetDeg = turretDeg;
            latchedInRange = true;
        } else if (!latchedInRange) {
            targetDeg = clamp(
                    turretDeg,
                    TURRET_MIN_TICKS / TICKS_PER_DEGREE,
                    TURRET_MAX_TICKS / TICKS_PER_DEGREE
            );
        }

        targetTicks = clampInt(
                (int) (targetDeg * TICKS_PER_DEGREE),
                TURRET_MIN_TICKS,
                TURRET_MAX_TICKS
        );
    }

    /* ================= MANUAL INPUT ================= */
    private void handleTurretInputs() {

        if (gamepad2.dpad_right) targetDeg += 5;
        if (gamepad2.dpad_left)  targetDeg -= 5;

        if (gamepad2.y) targetDeg = 0;
        if (gamepad2.b) targetDeg = 90;
        if (gamepad2.x) targetDeg = -90;

        if (gamepad2.a) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            targetDeg = 0;
            lastError = 0;
            lastPower = 0;
            latchedInRange = false;
        }

        targetTicks = clampInt(
                (int) (targetDeg * TICKS_PER_DEGREE),
                TURRET_MIN_TICKS,
                TURRET_MAX_TICKS
        );
    }

    /* ================= TURRET PD ================= */
    private void updateTurretPID() {

        int currentTicks = turret.getCurrentPosition();
        double error = targetTicks - currentTicks;
        double derivative = error - lastError;

        double power = (TURRET_kP * error) + (TURRET_kD * derivative);

        if (Math.abs(error) > 5) {
            power += Math.signum(error) * TURRET_kF;
        }

        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        double delta = clamp(power - lastPower, -TURRET_SLEW, TURRET_SLEW);
        power = lastPower + delta;

        if ((power > 0 && currentTicks >= TURRET_MAX_TICKS) ||
                (power < 0 && currentTicks <= TURRET_MIN_TICKS)) {
            power = 0;
        }

        turret.setPower(power);

        lastError = error;
        lastPower = power;
    }

    /* ================= TELEMETRY ================= */
    private void sendTelemetry() {

        Pose p = follower.getPose();

        telemetry.addLine("==== PASSIVE GOAL ALIGN ====");
        telemetry.addData("Goal X", GOAL_X);
        telemetry.addData("Goal Y", GOAL_Y);
        telemetry.addData("Robot X", p.getX());
        telemetry.addData("Robot Y", p.getY());
        telemetry.addData("Robot Heading", Math.toDegrees(p.getHeading()));

        telemetry.addLine("==== TURRET ====");
        telemetry.addData("Target Deg", targetDeg);
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Current Ticks", turret.getCurrentPosition());
        telemetry.addData("Power", lastPower);

        telemetry.update();
    }

    /* ================= UTILS ================= */
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private int clampInt(int v, int min, int max) {
        return Math.max(min, Math.min(max, v));
    }

    private double normalizeDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
