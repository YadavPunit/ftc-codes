package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "auto_blue_long2", group = "Auto")
public class auto_blue_long2 extends LinearOpMode {

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;
    private Timer opmodeTimer;

    // ================= HARDWARE =================
    private DcMotorEx shooterL, shooterR;
    private DcMotor intake;
    private DcMotorEx turret;

    private Servo hood, left_ramp, right_ramp;

    // ================= CONSTANTS =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 60.0;
    static final double SHOOTER_kI = 0.0;
    static final double SHOOTER_kD = 6.0;
    static final double SHOOTER_kF = 12.0;

    @Override
    public void runOpMode() {

        // ---------- FOLLOWER ----------
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(
                new Pose(54, 7, Math.toRadians(90))
        );

        // ---------- HARDWARE INIT ----------
        shooterL = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterR = hardwareMap.get(DcMotorEx.class, "right_shooting");
        turret   = hardwareMap.get(DcMotorEx.class, "turret");
        intake   = hardwareMap.get(DcMotor.class, "intake"); // ✅ FIXED

        hood = hardwareMap.get(Servo.class, "hood");
        left_ramp = hardwareMap.get(Servo.class, "left_ramp");
        right_ramp = hardwareMap.get(Servo.class, "right_ramp");

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF
        );
        shooterR.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF
        );

        // ---------- TURRET LOCK ----------
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0);

        // ---------- SERVO POSITIONS ----------
        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);

        hood.setPosition(0.28);

        // ---------- PATHS ----------
        paths = new Paths(follower);
        opmodeTimer = new Timer();


        telemetry.addLine("Auto Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ================= AUTO =================

        setShooterRPM(3790);
        runPath(paths.Path1);

        intake.setPower(0.4);

        sleep(1000);

        left_ramp.setPosition(0.6);
        right_ramp.setPosition(0.4);

        sleep(1000);

        setShooterRPM(3790);
        sleep(1500);

        setShooterRPM(0);
        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);

        intake.setPower(0.4);

        runPath(paths.Path2);
        setShooterRPM(3790);

        runPath(paths.Path3);

        left_ramp.setPosition(0.6);
        right_ramp.setPosition(0.4);
        sleep(1500);
        setShooterRPM(0);
        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);

        runPath(paths.Path4);
        setShooterRPM(3790);

        runPath(paths.Path5);

        left_ramp.setPosition(0.6);
        right_ramp.setPosition(0.4);
        sleep(1500);
        setShooterRPM(0);
        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);








        telemetry.addLine("Auto Complete");
        telemetry.update();
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

    // ================= SHOOTER RPM =================
    private void setShooterRPM(double rpm) {
        double ticksPerSecond = (rpm / 60.0) * TICKS_PER_REV;
        shooterL.setVelocity(ticksPerSecond);
        shooterR.setVelocity(ticksPerSecond);
    }

    // ================= PATHS =================
    public static class Paths {

        public PathChain Path1, Path2,Path3,Path4,Path5;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(54, 7),
                                    new Pose(65.196, 15.813)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(108)
                    )
                    .build();
            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(65.196, 15.813),
                                    new Pose(60.299, 38.005),
                                    new Pose(24.822, 35.935)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(108), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20, 35.935),

                                    new Pose(57, 20)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();
            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.551, 20.103),
                                    new Pose(65.720, 64.374),
                                    new Pose(24.000, 60.336)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(128), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.963, 60.336),

                                    new Pose(57.645, 20.636)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(128))

                    .build();



        }
    }
}