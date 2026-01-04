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

@Autonomous(name = "auto_test", group = "Auto")
public class auto_test extends LinearOpMode {

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;

    // ================= HARDWARE =================
    private DcMotor intake;
    private DcMotorEx shooting, shooting1;
    private Servo hood, leftRamp, rightRamp;
    private Limelight3A limelight;

    // ================= CONSTANTS =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 60.0;
    static final double SHOOTER_kI = 0.0;
    static final double SHOOTER_kD = 6.0;
    static final double SHOOTER_kF = 12.0;

    static final double SHOOTER_ACCEL_RPM_PER_SEC = 2500;
    static final double RPM_TOLERANCE = 100;
    static final long FEED_TIME_MS = 2000;

    static final double ALIGN_KP = 0.03;
    static final double ALIGN_KD = 0.001;
    static final double TY_OFFSET = 8;

    static final double RAMP_DOWN = 0.37;
    static final double RAMP_UP   = 0.65;

    static final double HOOD_MIN = 0.30;
    static final double HOOD_MAX = 1.00;

    static final double INTAKE_SHOOT = -0.8;

    // ================= STATE =================
    private double lastAlignError = 0;
    private double currentRPM = 0;
    private double lastShooterTime = 0;

    // ================= INIT =================
    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(65.7196, 7.4018, Math.toRadians(90))
        );

        intake    = hardwareMap.get(DcMotor.class, "intake");
        shooting  = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooting1 = hardwareMap.get(DcMotorEx.class, "right_shooting");

        leftRamp  = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");
        hood      = hardwareMap.get(Servo.class, "hood");

        shooting1.setDirection(DcMotor.Direction.REVERSE);

        shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooting.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF
        );
        shooting1.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF
        );

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        setRampPosition(RAMP_DOWN);
        setHoodPosition(0.50);

        paths = new Paths(follower);

        telemetry.addLine("Auto Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        limelight.start();
        sleep(600);

        runPath(paths.Path1);

        alignAndShootAuto(4500);

        intake.setPower(-0.65);

        runPath(paths.path2);

        telemetry.addLine("Auto Complete");
        telemetry.update();
    }

    // ================= ALIGN + SHOOT CONFIG =================
    private void alignAndShootAuto(long timeoutMs) {

        long startTime = System.currentTimeMillis();
        long feedStart = 0;

        boolean shooterReady = false;

        currentRPM = 0;
        lastShooterTime = getRuntime();

        while (opModeIsActive()
                && System.currentTimeMillis() - startTime < timeoutMs) {

            follower.update();

            LLResult ll = limelight.getLatestResult();
            if (ll == null || !ll.isValid()
                    || Double.isNaN(ll.getTx())
                    || Double.isNaN(ll.getTy())) {
                follower.setTeleOpDrive(0, 0, 0, true);
                continue;
            }

            // ---------- ALIGN ----------
            double error = -(ll.getTy() + TY_OFFSET);
            double derivative = error - lastAlignError;
            lastAlignError = error;
            double turn = ALIGN_KP * error + ALIGN_KD * derivative;
            follower.setTeleOpDrive(0, 0, turn, true);

            // ---------- RPM + HOOD ----------
            double tx = clamp(ll.getTx(), -4.97, 14.0);
            double targetRPM = map(tx, -4.97, 14.0, 3100, 4050);
            targetRPM = clamp(targetRPM, 0, 4200);

            double smoothRPM = rampRPM(targetRPM);

            shooting.setVelocity(rpmToTicks(smoothRPM));
            shooting1.setVelocity(rpmToTicks(smoothRPM));
            setHoodPosition(map(tx, -4.97, 14.0, 0.95, 0.45));

            // ---------- READY CHECK ----------
            if (!shooterReady &&
                    Math.abs(smoothRPM - targetRPM) < RPM_TOLERANCE) {
                shooterReady = true;
                feedStart = System.currentTimeMillis();
            }

            // ---------- FEED ----------
            if (shooterReady) {
                intake.setPower(INTAKE_SHOOT);
                setRampPosition(RAMP_UP);

                if (System.currentTimeMillis() - feedStart > FEED_TIME_MS) {
                    break;
                }
            }
        }

        // ---------- STOP ----------
        follower.setTeleOpDrive(0, 0, 0, true);
        shooting.setVelocity(0);
        shooting1.setVelocity(0);
        intake.setPower(0);
        setRampPosition(RAMP_DOWN);
        currentRPM = 0;
    }

    // ================= PATH RUNNER =================
    private void runPath(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    // ================= HELPERS =================
    private double rampRPM(double targetRPM) {
        double now = getRuntime();
        double dt = Math.min(now - lastShooterTime, 0.05);
        lastShooterTime = now;

        double step = SHOOTER_ACCEL_RPM_PER_SEC * dt;

        if (targetRPM > currentRPM)
            currentRPM = Math.min(currentRPM + step, targetRPM);
        else
            currentRPM = Math.max(currentRPM - step, targetRPM);

        return currentRPM;
    }

    private double map(double x, double inMin, double inMax,
                       double outMin, double outMax) {
        return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double rpmToTicks(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    private void setRampPosition(double pos) {
        pos = clamp(pos, RAMP_DOWN, RAMP_UP);
        leftRamp.setPosition(pos);
        rightRamp.setPosition(1.0 - pos);
    }

    private void setHoodPosition(double pos) {
        hood.setPosition(clamp(pos, HOOD_MIN, HOOD_MAX));
    }

    // ================= PATH DEFINITIONS =================
    public static class Paths {

        public PathChain Path1,path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(65.720, 7.402),
                                    new Pose(62.131, 81.196)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(135)
                    )
                    .build();
        }
    }
}
