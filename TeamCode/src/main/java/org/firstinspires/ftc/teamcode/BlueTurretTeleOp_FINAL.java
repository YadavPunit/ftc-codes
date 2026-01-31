package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Blue Turret Field Centric FINAL", group = "Competition")
public class BlueTurretTeleOp_FINAL extends OpMode {

    /* ================= HARDWARE ================= */
    private DcMotorEx turret;
    private Follower follower;

    /* ================= START POSE ================= */
    private static final Pose START_POSE =
            new Pose(37.234, 71.776, Math.toRadians(90));

    /* ================= BLUE GOAL ================= */
    private static final double GOAL_X = 14;
    private static final double GOAL_Y = 131;

    /* ================= TURRET CONSTANTS ================= */
    private static final double TICKS_PER_REV = 28;      // REV HD Hex
    private static final double GEAR_RATIO = 18.0;      // 5:1 × 3.6
    private static final double TICKS_PER_DEGREE =
            (TICKS_PER_REV * GEAR_RATIO) / 360.0;        // 1.4

    private static final double TURRET_LIMIT_DEG = 125.0;

    /* ================= PD TUNING ================= */
    public static double kP = 0.018;
    public static double kD = 0.002;
    public static double maxPower = 0.6;

    private double lastError = 0;

    @Override
    public void init() {

        /* -------- TURRET MOTOR -------- */
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        /* -------- PEDRO FOLLOWER (PROJECT-CORRECT) -------- */
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
    }

    @Override
    public void loop() {

        /* -------- UPDATE ROBOT POSE -------- */
        follower.update();
        Pose pose = follower.getPose();

        double botX = pose.getX();
        double botY = pose.getY();
        double botHeadingRad = pose.getHeading();

        /* -------- LEFT BUMPER → AUTO AIM -------- */
        if (gamepad1.left_bumper) {
            aimTurret(botX, botY, botHeadingRad);
        } else {
            turret.setPower(0);
            lastError = 0;
        }

        /* -------- TELEMETRY -------- */
        telemetry.addData("Bot X", botX);
        telemetry.addData("Bot Y", botY);
        telemetry.addData("Heading (deg)", Math.toDegrees(botHeadingRad));
        telemetry.addData("Turret Angle (deg)", getTurretAngleDeg());
        telemetry.update();
    }

    /* ================= TURRET AIM LOGIC ================= */

    private void aimTurret(double botX, double botY, double botHeadingRad) {

        // Field-centric angle robot → blue goal
        double fieldAngleDeg = Math.toDegrees(
                Math.atan2(GOAL_Y - botY, GOAL_X - botX)
        );

        // Convert to robot-relative turret angle
        double targetDeg = normalize(
                fieldAngleDeg - Math.toDegrees(botHeadingRad)
        );

        // Mechanical limits
        targetDeg = Range.clip(
                targetDeg,
                -TURRET_LIMIT_DEG,
                TURRET_LIMIT_DEG
        );

        double currentDeg = getTurretAngleDeg();
        double error = normalize(targetDeg - currentDeg);
        double derivative = error - lastError;

        double power = (kP * error) + (kD * derivative);
        turret.setPower(Range.clip(power, -maxPower, maxPower));

        lastError = error;
    }

    /* ================= UTIL ================= */

    private double getTurretAngleDeg() {
        return turret.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    private double normalize(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
