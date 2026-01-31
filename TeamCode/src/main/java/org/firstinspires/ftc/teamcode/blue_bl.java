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

@Autonomous(name = "blue_bl", group = "Auto")
public class blue_bl extends LinearOpMode {

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;

    // ================= HARDWARE =================
    private DcMotorEx shooterL, shooterR;
    private DcMotor intake;
    private DcMotorEx turret;
    private Servo hood, left_ramp, right_ramp;

    // ================= LIMELIGHT =================
    private Limelight3A limelight;

    // ================= SHOOTER =================
    static final double TICKS_PER_REV = 28.0;

    static final double SHOOTER_kP = 98;
    static final double SHOOTER_kI = 0;
    static final double SHOOTER_kD = 14;
    static final double SHOOTER_kF = 18;

    // ================= TURRET LIMITS =================
    static final int TURRET_MIN_TICKS = -175;
    static final int TURRET_MAX_TICKS =  175;

    // ================= TURRET LIMELIGHT CONTROL =================
    static final double TURRET_KP = 0.035;
    static final double TURRET_MAX_POWER = 0.6;
    static final double TURRET_DEADBAND = 0.9;

    // ================= INIT =================
    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(54.206, 6.1, Math.toRadians(180)));

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
        turret.setPower(0);

        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);
        hood.setPosition(0.34);

        paths = new Paths(follower);

        telemetry.addLine("Auto Ready (WITH LIMELIGHT)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ================= AUTO =================
        intake.setPower(0.45);
        setShooterRPM(3790);
        sleep(1000);

        runPath(paths.Path1, 0.65);
        alignAndShoot(3790, true, -2.5);

        runPath(paths.Path2, 0.65);
        runPath(paths.Path3, 0.55);
        alignAndShoot(3790, true, 2.5);

        runPath(paths.Path4, 0.65);
        hood.setPosition(0.56);
        setShooterRPM(2700);
        runPath(paths.Path5, 0.55);
        alignAndShoot(2780, true, 0);


        runPath(paths.Path6, 0.65);
        runPath(paths.Path7, 0.55);
        alignAndShoot(2780, true, 0);

        runPath(paths.Path8, 0.65);

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }

    // ================= SHOOT =================
    private void alignAndShoot(double rpm, boolean turretAlign, double tyOffset) {

        if (turretAlign) {
            alignTurretWithLimelight(tyOffset);
        }

        left_ramp.setPosition(0.55);
        right_ramp.setPosition(0.45);
        sleep(1500);

        left_ramp.setPosition(0.3);
        right_ramp.setPosition(0.7);
    }

    // ================= TURRET ALIGN =================
    private void alignTurretWithLimelight(double tyOffset) {

        long timeout = System.currentTimeMillis() + 1200;

        while (opModeIsActive() && System.currentTimeMillis() < timeout) {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                turret.setPower(0);
                continue;
            }

            double ty = result.getTy() + tyOffset;

            if (Math.abs(ty) < TURRET_DEADBAND) {
                turret.setPower(0);
                break;
            }

            double power = ty * TURRET_KP;
            power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));

            int pos = turret.getCurrentPosition();

            if ((pos <= TURRET_MIN_TICKS && power < 0) ||
                    (pos >= TURRET_MAX_TICKS && power > 0)) {
                power = 0;
            }

            turret.setPower(power);

            telemetry.addData("TY", ty);
            telemetry.addData("Turret Pos", pos);
            telemetry.addData("Power", power);
            telemetry.update();
        }

        turret.setPower(0);
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
                Path6, Path7, Path8;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(new Pose(54.206, 6.1),
                                    new Pose(59.140, 14.467)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.140, 14.467),
                                    new Pose(59.393, 37.668),
                                    new Pose(61.682, 37.458),
                                    new Pose(16.168, 36.925)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.168, 36.925),
                                    new Pose(59.178, 14.411)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.178, 14.411),
                                    new Pose(61.930, 65.972),
                                    new Pose(54.509, 64.093),
                                    new Pose(17.991, 62.374)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(17.991, 62.374),
                                    new Pose(50.836, 65.047),
                                    new Pose(41.701, 95.308)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(41.701, 95.308),
                                    new Pose(43.206, 83.542),
                                    new Pose(47.019, 83.374),
                                    new Pose(18.935, 83.963)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.935, 83.963),
                                    new Pose(41.486, 95.411)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.486, 95.411),

                                    new Pose(37.234, 71.776)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();
        }
    }
}
