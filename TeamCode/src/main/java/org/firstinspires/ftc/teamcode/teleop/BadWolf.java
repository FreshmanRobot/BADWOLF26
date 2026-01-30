package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="A BadWolf Official ", group="Linear OpMode")
public class BadWolf extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotorEx shooterEx;
    private DcMotor shooter2, intakeMotor;
    private Servo clawServo = null;
    private Servo leftHoodServo = null;
    private Servo gateServo = null;
    

    // Gate/Intake constants - UPDATED POSITIONS
    private static final double GATE_OPEN = 0.28;
    private static final double GATE_CLOSED = 0.60;
    private static final long INTAKE_DURATION_MS = 1200;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 100;
    private static final double INTAKE_SEQUENCE_POWER = 1.0;

    // shooter state (we will drive it via FlywheelController)
    private FlywheelController flywheel;

    private double targetRPM = 3000;

    private double rpmScale = 1.0;

    private boolean backPressedLast = false;
    private boolean aPressedLast = false;
    private boolean xPressedLast = false;
    private boolean yPressedLast = false;

    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean dpadUpLast = false;
    private boolean bPressedLast = false;

    private boolean atTargetLast = false;
    private boolean rumbling = false;
    private long rumbleEndTimeMs = 0L;
    private static final long RUMBLE_DURATION_MS = 1000L;

    // hood/claw timing
    private double leftHoodPosition = 0.9;

    // Claw constants
    private static final double CLAW_OPEN = 0.2;
    private static final double CLAW_CLOSED = 0.63;
    private static final long CLAW_CLOSE_MS = 500L;

    // precision driving
    private double driveScale = 1.0;
    private static final double PRECISION_SCALE = 0.25;
    private boolean leftBumperLast = false;
    private boolean rightBumperLast = false;

    // ----- IMU align control internals -----
    // These allow for braking strength and tolerance logic
    // ============ IMU Align Logic ============
    private IMU imu = null;
    private double imuAlignAngle = 0.0;
    private boolean xImuAlignActive = false; // acts as an "aligning" state for X button/IMU align

    private double lastImuError = 0;
    private double lastImuTime = 0;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        // Obtain shooter as DcMotorEx (FlywheelController expects DcMotorEx primary)
        shooterEx = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        leftHoodServo = hardwareMap.get(Servo.class, "hoodServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooterEx.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);

        // let FlywheelController handle encoder modes; keep shooter2 as no-encoder mirror
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        GateController gateController;
        ClawController clawController;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize controllers
        targetRPM = 3000;
        clawServo.setPosition(0.0);
        leftHoodServo.setPosition(leftHoodPosition);

        clawController = new ClawController(clawServo, CLAW_OPEN, CLAW_CLOSED, CLAW_CLOSE_MS);
        gateController = new GateController(
                gateServo, intakeMotor,
                GATE_OPEN, GATE_CLOSED,
                INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                INTAKE_SEQUENCE_POWER
        );


        // Initialize at the Open position
        gateServo.setPosition(GATE_CLOSED);

        // Try to obtain a voltage sensor if available (optional)
        VoltageSensor voltageSensor = null;
        try {
            if (hardwareMap.voltageSensor.iterator().hasNext()) {
                voltageSensor = hardwareMap.voltageSensor.iterator().next();
            }
        } catch (Exception ignored) {}

        // Create FlywheelController: primary (DcMotorEx), secondary (DcMotor), telemetry, voltageSensor (may be null)
        flywheel = new FlywheelController(shooterEx, shooter2, telemetry, voltageSensor);
        flywheel.setTargetRpm(targetRPM);
        flywheel.setShooterOn(true);

        // ========== IMU INIT ==========
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
            imu.initialize(parameters);
            imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        } catch (Exception ex) {
            telemetry.addData("IMU", "ERROR initializing IMU: " + ex.getMessage());
            telemetry.update();
        }
        waitForStart();

        // IMU align state variables
        ElapsedTime alignTimer = new ElapsedTime();

        // IMU align controller parameters
        final double kP = 0.02;      // Proportional gain
        final double kD = 0.003;     // Derivative gain for damping
        final double minPower = 0.07;  // Minimum power to overcome friction
        final double angleTolerance = 1.5; // Degrees, when to stop
        final double velocityTolerance = 0.2; // Not used as encoder velocity not available on DcMotor (non-Ex)
        final double alignTimeout = 1.5; // seconds

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // ---------- Relocalize (A) - reference heading store/restore --------------
            boolean aNow = gamepad1.a || gamepad2.a;
            if (aNow && !aPressedLast && imu != null) {
                imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }
            aPressedLast = aNow;

            // ---------- IMU Align (X) ---------------
            boolean xNow = gamepad1.x || gamepad2.x;
            // Enter IMU align on X pressed (not held)
            if (xNow && !xPressedLast && imu != null) {
                xImuAlignActive = true;
                alignTimer.reset();
                lastImuError = 0;
                lastImuTime = alignTimer.seconds();
            }
            xPressedLast = xNow;
            if (xImuAlignActive && imu != null) {
                // --- Improved align logic with P+D controller and braking ---
                double currentTime = alignTimer.seconds();
                double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double error = imuAlignAngle - imuAngle;

                // Derivative calculation for braking (damping)
                double deltaTime = currentTime - lastImuTime;
                double derivative = 0;
                if (deltaTime > 0) derivative = (error - lastImuError) / deltaTime;

                double turnPower = kP * error + kD * derivative;

                // Clamp for minimum power for large error, but stop for small error
                if (Math.abs(turnPower) < minPower && Math.abs(error) > angleTolerance) {
                    turnPower = Math.signum(turnPower) * minPower;
                }
                if (Math.abs(error) < angleTolerance) {
                    turnPower = 0;
                }

                // Apply rotation power
                frontLeftDrive.setPower(-turnPower);
                backLeftDrive.setPower(-turnPower);
                frontRightDrive.setPower(turnPower);
                backRightDrive.setPower(turnPower);

                telemetry.addData("IMU Angle", imuAngle);
                telemetry.addData("Target Angle", imuAlignAngle);
                telemetry.addData("Error", error);
                telemetry.addData("Turn Power", turnPower);
                telemetry.update();

                // Stop aligning if tolerance reached or timeout
                if (Math.abs(error) < angleTolerance || alignTimer.seconds() > alignTimeout) {
                    // Brake robot at heading
                    frontLeftDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backRightDrive.setPower(0);
                    sleep(150); // Brief brake to stabilize
                    xImuAlignActive = false;
                }
                lastImuError = error;
                lastImuTime = currentTime;
                continue; // skip rest of loop when aligning
            }
            // DRIVE LOGIC
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            boolean leftBumperNow = gamepad1.left_bumper || gamepad2.left_bumper;
            if (leftBumperNow && !leftBumperLast) driveScale = PRECISION_SCALE;
            leftBumperLast = leftBumperNow;

            boolean rightBumperNow = gamepad1.right_bumper || gamepad2.right_bumper;
            if (rightBumperNow && !rightBumperLast) driveScale = 1.0;
            rightBumperLast = rightBumperNow;

            frontLeftDrive.setPower(frontLeftPower * driveScale);
            backLeftDrive.setPower(backLeftPower * driveScale);
            frontRightDrive.setPower(frontRightPower * driveScale);
            backRightDrive.setPower(backRightPower * driveScale);

            // SHOOTER CONTROLS (same gamepad mapping preserved)
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) {
                shooterEx.setDirection(DcMotor.Direction.FORWARD);
                shooter2.setDirection(DcMotor.Direction.REVERSE);
                flywheel.toggleShooterOn();
            }
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) {
                targetRPM = Math.max(0.0, targetRPM - 100.0);
                flywheel.setTargetRpm(targetRPM);
            }
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) {
                // Respect FlywheelController's MAX_RPM by clamping
                targetRPM = Math.min(FlywheelController.MAX_RPM, targetRPM + 100.0);
                flywheel.setTargetRpm(targetRPM);
            }
            dpadRightLast = dpadRightNow;

            boolean dpadUpNow = gamepad1.dpad_up || gamepad2.dpad_up;
            if (dpadUpNow && !dpadUpLast) {
                shooterEx.setDirection(DcMotor.Direction.REVERSE);
                shooter2.setDirection(DcMotor.Direction.FORWARD);
                flywheel.toggleShooterOn();
            }
            dpadUpLast = dpadUpNow;

            // INTAKE (transfer motor removed)
            if (!gateController.isBusy()) {
                if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                    intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                    intakeMotor.setPower(1.0);
                } else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                    intakeMotor.setDirection(DcMotor.Direction.REVERSE);
                    intakeMotor.setPower(1.0);
                } else {
                    intakeMotor.setPower(0.0);
                }
            }

            // GATE TOGGLE (B Button - Gamepad 1 or 2)
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && !gateController.isBusy()) {
                gateController.toggleGate();
            }
            bPressedLast = bNow;

            // INTAKE AUTO SEQUENCE (Y Button)
            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast && !gateController.isBusy()) {
                gateController.startIntakeSequence(nowMs);
            }
            yPressedLast = yNow;

            // Update controllers
            boolean shouldTriggerClaw = gateController.update(nowMs);
            if (shouldTriggerClaw) clawController.trigger(nowMs);

            // --- Flywheel PIDF update (replaces manual PID-ish section) ---
            // Keep targetRPM in sync with flywheel
            flywheel.setTargetRpm(targetRPM);
            flywheel.update();

            // Rumble at target using FlywheelController isAtTarget()
            boolean atTargetNow = flywheel.isAtTarget();
            if (atTargetNow && !atTargetLast) {
                rumbling = true;
                rumbleEndTimeMs = nowMs + RUMBLE_DURATION_MS;
                gamepad1.rumble((int) RUMBLE_DURATION_MS);
                gamepad2.rumble((int) RUMBLE_DURATION_MS);
            }
            atTargetLast = atTargetNow;

            // CLAW MANUAL (X Button)
            boolean backNow = gamepad1.back || gamepad2.back;
            if (xNow && !backPressedLast) clawController.trigger(nowMs);
            backPressedLast = backNow;
            clawController.update(nowMs);

            // TELEMETRY
            telemetry.addData("Gate Pos", gateServo.getPosition());
            telemetry.addData("RPM", "%.1f", flywheel.getCurrentRPM());
            telemetry.addData("Target RPM", "%.1f", flywheel.getTargetRpm());
            telemetry.addData("Shooter Power", "%.3f", flywheel.getLastAppliedPower());
            telemetry.addData("PIDF Mode", flywheel.isUsingFarCoefficients() ? "FAR" : "CLOSE");
            telemetry.update();
        }
    }
}