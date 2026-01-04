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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "test1")
public class test1 extends OpMode {

    private Follower follower;
    private TelemetryManager telemetryM;
    private DcMotor intake;
    private DcMotor shooting;
    private DcMotor shooting1;


    private boolean autoAlign = false;
    private boolean slowMode = false;
    private boolean automatedDrive = false;
    private double slowMultiplier = 0.5;

    private Servo ramp;

    public static Pose startingPose;
    private Supplier<PathChain> pathChain;

    // ====== TOGGLE FUNCTIONALITY ======
    private boolean intakeOn = false;
    private boolean shootingOn = false;

    private boolean prevSquare = false;
    private boolean prevCircle = false;
    // =================================

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        startingPose = new Pose(38.57943925233644,33.644859813084096 , Math.toRadians(90));
        follower.setStartingPose(startingPose);
        follower.update();

        shooting = hardwareMap.get(DcMotorEx.class, "shooting");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooting = hardwareMap.get(DcMotor.class, "shooting");
        shooting1 = hardwareMap.get(DcMotor.class, "shooting1");
        ramp = hardwareMap.get(Servo.class, "ramp1");

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

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
        handleMotorToggles();
        handleDriving();
        handleAutomation();
        updateTelemetry();
    }

    private void handleToggles() {
        if (gamepad1.right_bumper) slowMode = !slowMode;
        if (gamepad1.dpad_up) slowMultiplier += 0.25;
        if (gamepad1.dpad_down) slowMultiplier = Math.max(0.25, slowMultiplier - 0.25);
    }

    private void handleMotorToggles() {

        // Square → Intake toggle
        if (gamepad2.square && !prevSquare) {
            intakeOn = !intakeOn;
        }
        prevSquare = gamepad2.square;
        intake.setPower(intakeOn ? -1 : 0);

        // Circle → Shooting toggle
        if (gamepad2.circle && !prevCircle) {
            shootingOn = !shootingOn;
        }
        prevCircle = gamepad2.circle;
        shooting.setPower(shootingOn ? -1.0 : 0);


        shooting1.setPower(shootingOn ? 1.0 : 0);

    }

    private void handleDriving() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // ===== LEFT TRIGGER LINKED CONTROL (ADDED) =====
        double trigger = gamepad2.left_trigger;
        if (trigger > 0.05) {
            intake.setPower(trigger * 0.7);        // positive
            shooting.setPower(-trigger * 0.4);     // negative
        }
        // ==============================================

        if (gamepad2.triangle) {
            ramp.setPosition(0.6);
        } else if (gamepad2.cross) {
            ramp.setPosition(0.92);
        }

        if (!automatedDrive) {
            double mult = slowMode ? slowMultiplier : 1.0;
            follower.setTeleOpDrive(forward * mult, strafe * mult, turn * mult, true);
        }
    }

    private void handleAutomation() {
        if (gamepad1.a && !automatedDrive) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.b || !follower.isBusy())) {
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
