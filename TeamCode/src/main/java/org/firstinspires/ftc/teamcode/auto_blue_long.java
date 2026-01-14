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

@Autonomous(name = "blue_back_right_align_shoot1", group = "Auto")
public class auto_blue_long extends LinearOpMode {

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;
    private Timer opmodeTimer;

    // ================= HARDWARE =================
    private DcMotor intake;
    private DcMotorEx shooterL, shooterR;
    private Servo hood, leftRamp, rightRamp;
    private Limelight3A limelight;

    // ================= CONSTANTS (FROM TELEOP test14) =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 60.0;
    static final double SHOOTER_kI = 0.0;
    static final double SHOOTER_kD = 6.0;
    static final double SHOOTER_kF = 12.0;

    static final double LONG_RPM  = 3790;
    static final double LONG_HOOD = 0.28;

    static final double RAMP_OPEN_LEFT = 0.3;
    static final double RAMP_CLOSE_LEFT = 0.6;

    static final double INTAKE_FEED = -0.5;

    static final double HOOD_MIN = 0.2;
    static final double HOOD_MAX = 0.8;

    // ================= INIT =================
    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(65.7196, 7.4018, Math.toRadians(90))
        );

        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooterL = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterR = hardwareMap.get(DcMotorEx.class, "right_shooting");

        shooterR.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF
        );
        shooterR.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF
        );

        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(HOOD_MIN, HOOD_MAX);
        hood.setPosition(0.5);

        leftRamp  = hardwareMap.get(Servo.class, "left_ramp");
        rightRamp = hardwareMap.get(Servo.class, "right_ramp");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        setRamp(RAMP_CLOSE_LEFT);
        setHood(0.5);

        paths = new Paths(follower);
        opmodeTimer = new Timer();

        telemetry.addLine("Auto Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        limelight.start();
        sleep(600);

        // ================= FOLLOW PATH =================
        runPath(paths.Path1);

        setRamp(RAMP_CLOSE_LEFT);
        sleep(800);

        alignAndShoot(1500);

        telemetry.addLine("Auto Complete");
        telemetry.update();
    }

    // ================= ALIGN & SHOOT =================
    private void alignAndShoot(long shootTimeMs) {

        double lastError = 0;
        long start = System.currentTimeMillis();

        while (opModeIsActive()
                && System.currentTimeMillis() - start < shootTimeMs) {

            follower.update();

            LLResult ll = limelight.getLatestResult();
            if (ll == null || !ll.isValid()) {
                setIntake(0);
                continue;
            }

            // ----- BOT ALIGN (TY PD) -----
            double error = -(ll.getTy() + 8);
            double derivative = error - lastError;
            lastError = error;

            double turn =
                    (0.03 * error) + (0.001 * derivative);

            follower.setTeleOpDrive(0, 0, turn, true);

            // ----- TELEOP VALUES -----
            setShooterRPM(LONG_RPM);
            setHood(LONG_HOOD);

            // ----- FEED -----
            sleep(1200);
            setRamp(RAMP_OPEN_LEFT);
            setIntake(INTAKE_FEED);

            telemetry.addData("Align Error", error);
            telemetry.update();
        }

        // ----- STOP -----
        follower.setTeleOpDrive(0, 0, 0, true);
        setShooterRPM(0);
        setIntake(0);
        setRamp(RAMP_CLOSE_LEFT);
    }

    // ================= PATH RUNNER =================
    private void runPath(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData(
                    "heading",
                    Math.toDegrees(follower.getPose().getHeading())
            );
            telemetry.update();
        }
    }

    // ================= VALUE-BASED HELPERS =================
    private void setShooterRPM(double rpm) {
        double ticks = (rpm / 60.0) * TICKS_PER_REV;
        shooterL.setVelocity(ticks);
        shooterR.setVelocity(ticks);
    }

    private void setHood(double pos) {
        pos = Math.max(HOOD_MIN, Math.min(HOOD_MAX, pos));
        hood.setPosition(pos);
    }

    private void setRamp(double leftPos) {
        leftRamp.setPosition(leftPos);
        rightRamp.setPosition(1.0 - leftPos);
    }

    private void setIntake(double power) {
        intake.setPower(power);
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
