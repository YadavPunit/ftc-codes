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

@Autonomous(name = "blue_apex_code_short_triangle", group = "Auto")
public class blue_apex_code_short_triangle extends LinearOpMode {

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

    static final double SHOOTER_kP = 80;
    static final double SHOOTER_kI = 0;
    static final double SHOOTER_kD = 8;
    static final double SHOOTER_kF = 17;

    // ================= TURRET LIMITS =================
    static final int TURRET_MIN_TICKS = -200;
    static final int TURRET_MAX_TICKS =  200;

    // ================= INIT =================
    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(54.206, 7.103, Math.toRadians(90)));

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
        hood.setPosition(0.27);

        paths = new Paths(follower);

        telemetry.addLine("Auto Ready (NO LIMELIGHT)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ================= AUTO =================
        intake.setPower(0.65);
        setShooterRPM(3605);
        sleep(1000);
        runPath(paths.Path1, 0.6);
        alignAndShoot(3890);

        runPath(paths.Path2, 0.9);
        runPath(paths.Path3, 0.6);
        runPath(paths.Path4, 0.7);

        alignAndShoot(3890);

        runPath(paths.Path5, 0.9);
        runPath(paths.Path6, 0.6);
        runPath(paths.Path7, 0.7);

        alignAndShoot(3890);

        runPath(paths.Path8, 0.9);
        runPath(paths.Path9, 0.6);
        runPath(paths.Path10, 0.7);

        alignAndShoot(3890);


        runPath(paths.Path11, 0.65);
        intake.setPower(0);
        setShooterRPM(0);



        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }

    // ================= SHOOT =================
    private void alignAndShoot(double rpm) {



        left_ramp.setPosition(0.55);
        right_ramp.setPosition(0.45);
        sleep(1500);

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
                                    new Pose(54, 7.075),

                                    new Pose(64.972, 17.383)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(105))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(64.972, 17.383),

                                    new Pose(41.888, 35.654)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.888, 35.654),

                                    new Pose(18.617, 35.664)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.617, 35.664),

                                    new Pose(64.495, 18.009)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(118))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(64.495, 18.009),

                                    new Pose(40.374, 60.336)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.374, 60.336),

                                    new Pose(18.832, 59.804)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.832, 59.804),

                                    new Pose(64.150, 18.841)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(118))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(64.150, 18.841),

                                    new Pose(40, 84.776)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(37.748, 84.776),

                                    new Pose(18.645, 83.813)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.645, 83.813),

                                    new Pose(64.701, 17.981)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(118))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(69.374, 20.449),

                                    new Pose(35.430, 71.738)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(90))

                    .build();

        }
    }
}
