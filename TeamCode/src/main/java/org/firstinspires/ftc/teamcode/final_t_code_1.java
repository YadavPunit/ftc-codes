package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "final_t_code")
public class final_t_code_1 extends OpMode {

    private Follower follower;
    private TelemetryManager telemetryM;
    private Limelight3A limelight;

    private DcMotor intake;
    private DcMotor shooting;

    private boolean autoAlign = false;
    private boolean slowMode = false;
    private boolean automatedDrive = false;
    private double slowMultiplier = 0.4;
    private Servo shoot;
    private Servo ramp;

    public static Pose startingPose;
    private Supplier<PathChain> pathChain;

    // ====== ADDED FOR TOGGLE FUNCTIONALITY ======
    private boolean intakeOn = false;
    private boolean shootingOn = false;

    private boolean prevSquare = false;
    private boolean prevCircle = false;
    // ===========================================

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        startingPose = new Pose(56, 8, Math.toRadians(90));
        follower.setStartingPose(startingPose);
        follower.update();

        shooting = hardwareMap.get(DcMotorEx.class, "shooting");
        shoot = hardwareMap.get(Servo.class, "shoot");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooting = hardwareMap.get(DcMotor.class, "shooting");
        ramp = hardwareMap.get(Servo.class, "ramp1");

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(
                        new BezierLine(
                                follower::getPose,
                                new Pose(38.57943925233644, 33.644859813084096, Math.toRadians(90))
                        )
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(90),
                                1.0
                        )
                )
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        handleToggles();
        handleMotorToggles();   // ✅ ADDED
        handleDriving();
        handleAutomation();
        updateTelemetry();
    }

    private void handleToggles() {
        if (gamepad1.right_bumper) slowMode = !slowMode;
        if (gamepad1.x) slowMultiplier += 0.25;
        if (gamepad1.y) slowMultiplier = Math.max(0.25, slowMultiplier - 0.25);
    }

    // ====== ADDED METHOD (NO OTHER LOGIC TOUCHED) ======
    private void handleMotorToggles() {

        // Square → Intake toggle
        if (gamepad1.square && !prevSquare) {
            intakeOn = !intakeOn;
        }
        prevSquare = gamepad1.square;
        intake.setPower(intakeOn ? -0.7 : 0);



        // Circle → Shooting toggle
        if (gamepad1.circle && !prevCircle) {
            shootingOn = !shootingOn;
        }
        prevCircle = gamepad1.circle;
        shooting.setPower(shootingOn ? 1.0 : 0);
    }
    // ==================================================

    private void handleDriving() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        if (gamepad2.left_bumper) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double headingCorrection = Math.max(-0.3, Math.min(0.3, -tx * 0.005));
                turn = headingCorrection;
            }
        }

        if (gamepad1.triangle) {
            ramp.setPosition(0.92);
        } else if (gamepad1.cross) {
            ramp.setPosition(0.7);
        }

        if (!automatedDrive) {
            double mult = slowMode ? slowMultiplier : 1.0;
            follower.setTeleOpDrive(forward * mult, strafe * mult, turn * mult, true);
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();

            double servoPos = (ty - (-19)) / (5.6 - (-19));
            servoPos = 1 - servoPos;
            servoPos = Math.max(0, Math.min(1, servoPos));
            shoot.setPosition(servoPos);
        }
    }

    private void handleAutomation() {
        if (gamepad2.a && !automatedDrive) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad2.b || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
    }

    private void updateTelemetry() {
        telemetryM.debug("Pose", follower.getPose());
        telemetryM.debug("Velocity", follower.getVelocity());
        telemetryM.debug("Automated Drive", automatedDrive);
        telemetryM.debug("Auto Align", autoAlign);
        telemetryM.debug("Slow Mode", slowMode + " (" + slowMultiplier + ")");
    }
}
