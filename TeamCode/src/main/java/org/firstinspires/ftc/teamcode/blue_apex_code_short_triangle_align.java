package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blue_apex_code_short_triangle_align", group = "Auto")
public class blue_apex_code_short_triangle_align extends LinearOpMode {

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;

    // ================= HARDWARE =================
    private DcMotorEx shooterL, shooterR;
    private DcMotor intake;

    private Limelight3A limelight;
    private DcMotorEx turret;
    private Servo hood, left_ramp, right_ramp;

    // ================= SHOOTER =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 60;
    static final double SHOOTER_kI = 0;
    static final double SHOOTER_kD = 6;
    static final double SHOOTER_kF = 12;

    // ================= TURRET LIMITS =================
    static final double TURRET_kP = 0.03;
    static final double TURRET_kD = 0.004;
    static final double TURRET_DEADBAND = 0.15;
    static final double TURRET_MAX_POWER = 0.65;
    static final double TURRET_SLEW = 0.04;
    static final double TURRET_ALIGN_DAMP = 0.6;

    static final int TURRET_MIN_TICKS = -400;
    static final int TURRET_MAX_TICKS = 400;

    private double lastTurretError = 0;
    private double lastTurretPower = 0;

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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

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
        hood.setPosition(0.29);

        paths = new Paths(follower);

        telemetry.addLine("Auto Ready (NO LIMELIGHT)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ================= AUTO =================
        intake.setPower(0.65);
        setShooterRPM(3890);

        alignTurret(true);
        sleep(1000);
        runPath(paths.Path1, 0.65);
        alignAndShoot(3890);

        runPath(paths.Path2, 0.9);
        runPath(paths.Path3, 0.55);
        runPath(paths.Path4, 0.65);

        alignAndShoot(3890);

        runPath(paths.Path5, 0.9);
        runPath(paths.Path6, 0.55);
        runPath(paths.Path7, 0.65);

        alignAndShoot(3890);

        runPath(paths.Path8, 0.9);
        runPath(paths.Path9, 0.55);
        runPath(paths.Path10, 0.65);

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

    private void alignTurret(boolean enable) {

        LLResult ll = limelight.getLatestResult();
        if (!enable || ll == null || !ll.isValid()) {
            turret.setPower(0);
            lastTurretPower = 0;
            lastTurretError = 0;
            return;
        }

        double error = ll.getTy();

        if (Math.abs(error) < TURRET_DEADBAND) {
            turret.setPower(0);
            lastTurretPower = 0;
            return;
        }

        double derivative = error - lastTurretError;
        lastTurretError = error;

        double power = (TURRET_kP * error) + (TURRET_kD * derivative);
        power *= TURRET_ALIGN_DAMP;
        power = clamp(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        double delta = clamp(power - lastTurretPower, -TURRET_SLEW, TURRET_SLEW);
        power = lastTurretPower + delta;
        lastTurretPower = power;

        int pos = turret.getCurrentPosition();
        if ((power > 0 && pos >= TURRET_MAX_TICKS) ||
                (power < 0 && pos <= TURRET_MIN_TICKS)) {
            turret.setPower(0);
            return;
        }

        turret.setPower(power);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
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
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(64.972, 17.383),

                                    new Pose(41.888, 35.654)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

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
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(100))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(64.495, 18.009),

                                    new Pose(40.374, 60.336)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(108), Math.toRadians(180))

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
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(100))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(64.150, 18.841),

                                    new Pose(37.748, 84.776)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(108), Math.toRadians(180))

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
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(100))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(69.374, 20.449),

                                    new Pose(35.430, 71.738)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

        }
    }
}
