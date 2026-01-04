package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "red_back_right5", group = "Examples")
public class example_auto5_with_turret extends LinearOpMode {

    // ================= Drive =================
    private Follower follower;
    private Paths paths;

    // ================= Hardware =================
    private DcMotorEx shooting, shooting1;
    private DcMotor intake;
    private DcMotorEx turret;
    private Servo ramp, hood;

    private Limelight3A limelight;

    // ================= Vision =================
    private static final double TARGET_TY = 3.15;
    private static final double kP_TURN = 0.02;
    private static final double TURN_LIMIT = 0.35;

    // ================= Shooter =================
    private static final double MAX_RPM = 6000.0;
    private static final double RPM_BOOST = 500.0;
    private static final double RPM_TOLERANCE = 120;

    // ================= Turret =================
    private static final int TURRET_LEFT_LIMIT  = -300; // ~ -150°
    private static final int TURRET_RIGHT_LIMIT =  300; // ~ +150°
    private static final double TURRET_POWER = 0.25;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(89.495, 7.750, Math.toRadians(90)));

        shooting  = hardwareMap.get(DcMotorEx.class, "shooting");
        shooting1 = hardwareMap.get(DcMotorEx.class, "shooting1");
        intake    = hardwareMap.get(DcMotor.class, "intake");
        turret    = hardwareMap.get(DcMotorEx.class, "turret");
        ramp      = hardwareMap.get(Servo.class, "ramp1");
        hood      = hardwareMap.get(Servo.class, "hood");

        shooting.setDirection(DcMotor.Direction.REVERSE);

        shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ramp.setPosition(0.92);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        paths = new Paths(follower);

        waitForStart();
        if (isStopRequested()) return;

        runPath(paths.Path1);
        visionAlignAndShoot();

        runPath(paths.Path8);
    }

    private void runPath(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    // ================= TURRET + SHOOT =================
    private void visionAlignAndShoot() {

        long start = System.currentTimeMillis();

        // ---- Turret alignment (TY-based) ----
        while (opModeIsActive() && System.currentTimeMillis() - start < 1500) {

            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) continue;

            double ty = result.getTy();

            int turretPos = turret.getCurrentPosition();

            double turretCmd = 0;

            if (ty > 0 && turretPos > TURRET_LEFT_LIMIT) {
                turretCmd = -TURRET_POWER;
            }
            else if (ty < 0 && turretPos < TURRET_RIGHT_LIMIT) {
                turretCmd = TURRET_POWER;
            }

            turret.setPower(turretCmd);

            if (Math.abs(ty) < 0.2) break;
        }

        turret.setPower(0);

        // ---- SHOOTER LOGIC (UNCHANGED) ----
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        double tx = result.getTx();

        double hoodPos = map(tx, 4.44, 18.14, 0.8, 0.3);
        hoodPos = clip(hoodPos, 0.3, 0.8);
        hood.setPosition(hoodPos);

        double baseRPM = map(Math.abs(tx), 4.44, 18.14, 0.58 * MAX_RPM, 0.89 * MAX_RPM);
        baseRPM = clip(baseRPM, 0.58 * MAX_RPM, 0.89 * MAX_RPM);

        shooting.setVelocity(baseRPM);
        shooting1.setVelocity(baseRPM);

        sleep(800);

        ramp.setPosition(0.6);

        shooting.setVelocity(clip(baseRPM + RPM_BOOST, 0, MAX_RPM));
        shooting1.setVelocity(clip(baseRPM + RPM_BOOST, 0, MAX_RPM));

        sleep(1000);

        ramp.setPosition(0.92);
        shooting.setVelocity(0);
        shooting1.setVelocity(0);
    }

    private double map(double x, double inMin, double inMax,
                       double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin)
                / (inMax - inMin) + outMin;
    }

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ================= Paths =================
    public static class Paths {
        public PathChain Path1, Path8;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(89.720, 8.075),
                            new Pose(72.000, 72.224)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            Path8 = Path1;
        }
    }
}