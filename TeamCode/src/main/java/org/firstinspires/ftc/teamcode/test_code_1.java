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
@TeleOp(name = "final_teliop_align")
public class test_code_1 extends OpMode {

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

    @Override
    public void init() {
        // === Setup follower and pose ===
        follower = Constants.createFollower(hardwareMap);
        startingPose = new Pose(56, 8, Math.toRadians(90)); // Initial bot pose
        follower.setStartingPose(startingPose);
        follower.update();
        shooting = hardwareMap.get(DcMotorEx.class,"shooting");
        shoot = hardwareMap.get(Servo.class, "shoot");
        intake = hardwareMap.get(DcMotor.class,"intake");
        shooting = hardwareMap.get(DcMotor.class,"shooting");
        ramp = hardwareMap.get(Servo.class,"ramp1");
        // === Setup telemetry ===
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // === Setup Limelight ===
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // === Define target path (when A is pressed) ===
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(
                        new BezierLine(follower::getPose,
                                new Pose(38.57943925233644, 33.644859813084096, Math.toRadians(90)))
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 1.0)
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
        handleDriving();
        handleAutomation();
        updateTelemetry();
    }

    /** === Handle button toggles (auto-align, slow mode, etc.) === */
    private void handleToggles() {
        if (gamepad1.right_bumper) slowMode = !slowMode;
        if (gamepad1.x) slowMultiplier += 0.25;
        if (gamepad1.y) slowMultiplier = Math.max(0.25, slowMultiplier - 0.25);
    }




    /** === Handle manual TeleOp and auto-align driving === */
    private void handleDriving() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Auto-align if enabled
        if (gamepad2.left_bumper) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double headingCorrection = Math.max(-0.3, Math.min(0.3, -tx * 0.005));
                turn = headingCorrection;
                telemetryM.debug("Auto Align", "ON (tx=" + tx + ")");
            } else {
                telemetryM.debug("Auto Align", "No target detected");
            }
        }



        if (gamepad1.triangle) {
            ramp.setPosition(0.92);
        }
        else if (gamepad1.cross){
            ramp.setPosition(0.7);

        }

        if (!automatedDrive) {
            double mult = slowMode ? slowMultiplier : 1.0;
            follower.setTeleOpDrive(forward * mult, strafe * mult, turn * mult, true);
        }
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy(); // Vertical offset (°)

            // Map ty (5.6 → -19) to servo position (1 → 0)
            double servoPos = (ty - (-19)) / (5.6 - (-19));
            servoPos = 1 - servoPos;
            servoPos = Math.max(0, Math.min(1, servoPos)); // clamp to [0,1]

            shoot.setPosition(servoPos); // use local servo variable

            telemetry.addData("ty", ty);
            telemetry.addData("Servo Position", servoPos);
        }
        if (result != null && result.isValid()) {
            double ty = result.getTy(); // Vertical offset (°)

            // Map ty (5.6 → -19) to servo position (1 → 0)
            double servoPos = (ty - (-19)) / (5.6 - (-19));
            servoPos = 1 - servoPos;
            servoPos = Math.max(0, Math.min(1, servoPos)); // clamp to [0,1]

            shoot.setPosition(servoPos); // use local servo variable

            telemetry.addData("ty", ty);
            telemetry.addData("Servo Position", servoPos);
        }
        if (gamepad2.left_bumper) {
            if (result != null && result.isValid()) {
                double ty = result.getTy(); // Vertical offset (°)

                // Map ty (5.6 → -19) to velocity (30 → 400 RPM)
                double minTy = -19;
                double maxTy = 5.6;

                // Normalize ty within its range
                double normalized = (ty - minTy) / (maxTy - minTy);
                normalized = 1 - normalized;  // invert so lower ty → higher velocity
                normalized = Math.max(0, Math.min(1, normalized)); // clamp to [0,1]

                // Scale to 30 → 400 RPM range
                double targetRPM = 30 + normalized * (500 - 30);

                // Convert RPM to ticks per second for setVelocity()
                // Assuming 537.7 ticks per revolution for a 400 RPM motor (e.g., goBILDA 5202)
                double ticksPerRev = 394;
                double ticksPerSecond = (targetRPM / 60.0) * ticksPerRev;

                // Set motor velocity (using built-in velocity control)
                shooting.setPower(ticksPerSecond);

                telemetry.addData("ty", ty);
                telemetry.addData("Target RPM", targetRPM);
                telemetry.addData("Ticks per second", ticksPerSecond);
                telemetry.addData("Velocity (ticks/sec)", ticksPerSecond);
            }
        }
        else {
            shooting.setPower(0);


        }

    }

    /** === Handle automated movement when A or B pressed === */
    private void handleAutomation() {
        // Press A → move to target position
        if (gamepad2.a && !automatedDrive) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        // Stop automation when B is pressed or path completes
        if (automatedDrive && (gamepad2.b || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
    }

    /** === Telemetry info === */
    private void updateTelemetry() {
        telemetryM.debug("Pose", follower.getPose());
        telemetryM.debug("Velocity", follower.getVelocity());
        telemetryM.debug("Automated Drive", automatedDrive);
        telemetryM.debug("Auto Align", autoAlign);
        telemetryM.debug("Slow Mode", slowMode + " (" + slowMultiplier + ")");
    }
}
