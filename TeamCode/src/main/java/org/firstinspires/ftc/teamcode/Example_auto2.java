package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blue_back_right_align_shoot", group = "Auto")
public class Example_auto2 extends LinearOpMode {

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;
    private Timer opmodeTimer;

    // ================= HARDWARE =================
    private DcMotor intake;
    private DcMotorEx shooting, shooting1;
    private Servo hood, leftRamp, rightRamp;
    private Limelight3A limelight;

    // ================= CONSTANTS (FROM test6) =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 60.0;
    static final double SHOOTER_kI = 0.0;
    static final double SHOOTER_kD = 6.0;
    static final double SHOOTER_kF = 12.0;

    static final double ALIGN_KP = 0.03;
    static final double ALIGN_KD = 0.001;
    static final double TY_OFFSET = 8;

    static final double RAMP_MIN = 0.35;
    static final double RAMP_MAX = 0.70;
    static final double HOOD_MIN = 0.30;
    static final double HOOD_MAX = 1.00;

    static final double INTAKE_SHOOT = -0.8;

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

        setRampPosition(0.37);
        setHoodPosition(0.50);

        paths = new Paths(follower);
        opmodeTimer = new Timer();

        telemetry.addLine("Auto Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        limelight.start();
        sleep(600);

        // ================= ALIGN & SHOOT =================


        // ================= FOLLOW PATH =================
        runPath(paths.Path1);

        setRampPosition(0.37);

        sleep(1000);

        alignAndShoot(1500);

        telemetry.addLine("Auto Complete");
        telemetry.update();
    }

    // ================= ALIGN & SHOOT FUNCTION =================
    private void alignAndShoot(long shootTimeMs) {

        double lastError = 0;
        long start = System.currentTimeMillis();

        while (opModeIsActive()
                && System.currentTimeMillis() - start < shootTimeMs) {

            follower.update();

            LLResult ll = limelight.getLatestResult();
            if (ll == null || !ll.isValid()) {
                intake.setPower(0);
                continue;
            }

            // ---------- BOT ALIGN (TY PID) ----------
            double error = -(ll.getTy() + TY_OFFSET);
            double derivative = error - lastError;
            lastError = error;

            double turn =
                    (ALIGN_KP * error) + (ALIGN_KD * derivative);

            follower.setTeleOpDrive(0, 0, turn, true);

            // ---------- SHOOTER RPM + HOOD (TX MAP) ----------
            updateShooterAndHood(ll.getTx());

            // ---------- FEED ----------
            sleep(1500);
            intake.setPower(INTAKE_SHOOT);
            setRampPosition(0.65);

            telemetry.addData("Align Error", error);
            telemetry.update();
        }

        // ---------- STOP ----------
        follower.setTeleOpDrive(0, 0, 0, true);
        shooting.setPower(0);
        shooting1.setPower(0);
        intake.setPower(0);
        setRampPosition(0.37);
    }

    // ================= PATH RUNNER =================
    private void runPath(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading",
                    Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    // ================= HELPERS (FROM test6) =================
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

    // ================= LIMELIGHT TX MAPPING =================
    private void updateShooterAndHood(double tx) {

        tx = Math.max(-4.97, Math.min(14.0, tx));

        double rpm = map(tx, -4.97, 14.0, 3100, 4050);
        double hoodPos = map(tx, -4.97, 14.0, 0.95, 0.45);

        shooting.setVelocity(rpmToTicks(rpm));
        shooting1.setVelocity(rpmToTicks(rpm));
        setHoodPosition(hoodPos);
    }

    // ================= PATH DEFINITIONS =================
    public static class Paths {

        public PathChain Path1;

        public Paths(Follower follower) {

            Path1 = follower
                    .pathBuilder()
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
