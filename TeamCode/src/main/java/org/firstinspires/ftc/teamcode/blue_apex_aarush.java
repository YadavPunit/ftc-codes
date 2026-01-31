package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blue_apex_aarush", group = "Auto")
public class blue_apex_aarush extends LinearOpMode {

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;

    // ================= HARDWARE =================
    private DcMotorEx shooterL, shooterR;
    private DcMotor intake;
    private DcMotorEx turret;
    private Servo hood, left_ramp, right_ramp;

    // ================= SHOOTER =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 90;
    static final double SHOOTER_kI = 13;
    static final double SHOOTER_kD = 6;
    static final double SHOOTER_kF = 16;

    // ================= TURRET LIMITS =================
    static final int TURRET_MIN_TICKS = -200;
    static final int TURRET_MAX_TICKS =  200;

    // ================= INIT =================
    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21, 124, Math.toRadians(315)));

        shooterL = hardwareMap.get(DcMotorEx.class, "left_shooting");
        shooterR = hardwareMap.get(DcMotorEx.class, "right_shooting");
        turret   = hardwareMap.get(DcMotorEx.class, "turret");
        intake   = hardwareMap.get(DcMotor.class, "intake");

        hood = hardwareMap.get(Servo.class, "hood");
        left_ramp = hardwareMap.get(Servo.class, "left_ramp");
        right_ramp = hardwareMap.get(Servo.class, "right_ramp");

        shooterR.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterR.setVelocityPIDFCoefficients(
                SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0); // 🔒 turret locked

        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);
        hood.setPosition(0.34);

        paths = new Paths(follower);

        telemetry.addLine("Auto Ready (NO LIMELIGHT)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ================= AUTO =================
        intake.setPower(0.65);
        setShooterRPM(3100);
        sleep(1000);


        runPath(paths.Path1, 0.6);
        alignAndShoot(3100,0.65, 1500);




        runPath(paths.Path2, 0.9);
        runPath(paths.Path3, 0.55);
        runPath(paths.Path4, 0.65);
        runPath(paths.Path5, 0.55);
        sleep(1000);
        runPath(paths.Path6, 0.55);
        alignAndShoot(3040,0.65, 1500);

        runPath(paths.Path7, 0.9);
        runPath(paths.Path8, 0.65);
        runPath(paths.Path9, 0.55);

        alignAndShoot(3040,0.65, 1500);

        runPath(paths.Path10, 0.65);
        runPath(paths.Path11, 0.65);
        runPath(paths.Path12, 0.55);

        alignAndShoot(3620,0.55, 3000);






        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }

    // ================= SHOOT =================
    private void alignAndShoot(double rpm, double feed,double shoott) {

        setShooterRPM(rpm);
        intake.setPower(feed);
        left_ramp.setPosition(0.55);
        right_ramp.setPosition(0.45);
        sleep((long) shoott);

        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);

    }

    // ================= PATH =================
    private void runPath(PathChain path, double speed) {

        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }

        follower.setMaxPower(1.0);
    }

    // ================= SHOOTER =================
    private void setShooterRPM(double rpm) {
        double ticks = (rpm / 60.0) * TICKS_PER_REV;
        shooterL.setVelocity(ticks);
        shooterR.setVelocity(ticks);
    }

    // ================= PATHS =================
    public static class Paths {

        public PathChain Path1, Path2, Path3, Path4, Path5,
                Path6, Path7, Path8, Path9, Path10, Path11,Path12;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.2, 123.393),

                                    new Pose(63.178, 79.290)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(136))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(63.178, 79.290),

                                    new Pose(41.150, 60.449)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.150, 60.449),

                                    new Pose(19, 59.935)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.533, 59.888),

                                    new Pose(30.972, 70.561)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(360))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(30, 70.561),

                                    new Pose(16, 70.393)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(360))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.187, 70.617),

                                    new Pose(61.280, 82.589)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(136))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(66.280, 76.589),

                                    new Pose(43.121, 84.262)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))

                    .build();
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(35.944, 84.037),

                                    new Pose(21.290, 83.785)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.290, 83.785),

                                    new Pose(63.028, 79.178)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(63.028, 79.178),

                                    new Pose(45.336, 36.112)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.336, 36.112),

                                    new Pose(19.607, 35.888)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.607, 35.888),

                                    new Pose(66.907, 18.645)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(118))

                    .build();

        }
    }
}

