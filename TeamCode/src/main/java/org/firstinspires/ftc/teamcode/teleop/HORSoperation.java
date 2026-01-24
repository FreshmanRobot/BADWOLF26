// Java
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="HORS Operation ", group="Linear OpMode")
public class HORSoperation extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotorEx shooterEx;
    private DcMotor shooter2, intakeMotor;
    private Servo clawServo = null;
    private Servo leftHoodServo = null;
    private Servo gateServo = null;

    private static final double GATE_OPEN = 0.28;
    private static final double GATE_CLOSED = 0.73;
    private static final long INTAKE_DURATION_MS = 1200;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 100;
    private static final double INTAKE_SEQUENCE_POWER = -1.0;

    private FlywheelController flywheel;

    private double targetRPM = 3000;
    private boolean xPressedLast = false;
    private boolean yPressedLast = false;

    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean dpadUpLast = false;
    private boolean bPressedLast = false;

    private boolean atTargetLast = false;
    private static final long RUMBLE_DURATION_MS = 1000L;

    private double leftHoodPosition = 0.9;

    private static final double CLAW_OPEN = 0.2;
    private static final double CLAW_CLOSED = 0.63;
    private static final long CLAW_CLOSE_MS = 500L;

    private double driveScale = 1.0;
    private static final double PRECISION_SCALE = 0.25;
    private boolean leftBumperLast = false;
    private boolean rightBumperLast = false;

    // ============ Simplified IMU Align ============
    private IMU imu = null;
    private double imuAlignAngle = 0.0;
    private boolean aPressedLast = false;
    private boolean xImuAlignActive = false;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

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

        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        GateController gateController;
        ClawController clawController;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        targetRPM = 2900.0;
        clawServo.setPosition(0.0);
        leftHoodServo.setPosition(leftHoodPosition);

        clawController = new ClawController(clawServo, CLAW_OPEN, CLAW_CLOSED, CLAW_CLOSE_MS);
        gateController = new GateController(
                gateServo, intakeMotor,
                GATE_OPEN, GATE_CLOSED,
                INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                INTAKE_SEQUENCE_POWER
        );

        gateController.setGateClosed(false);
        gateServo.setPosition(GATE_OPEN);

        VoltageSensor voltageSensor = null;
        try {
            if (hardwareMap.voltageSensor.iterator().hasNext()) {
                voltageSensor = hardwareMap.voltageSensor.iterator().next();
            }
        } catch (Exception ignored) {}

        flywheel = new FlywheelController(shooterEx, shooter2, telemetry, voltageSensor);
        flywheel.setTargetRpm(targetRPM);
        flywheel.setShooterOn(true);

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

        // IMU align timer
        ElapsedTime alignTimer = new ElapsedTime();

        // Align params: simple P only, no braking/derivative/min power
        final double kP = 0.02;
        final double angleTolerance = 2.5;
        final double alignTimeout = 0.5;

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // Store reference heading on A
            boolean aNow = gamepad1.a || gamepad2.a;
            if (aNow && !aPressedLast && imu != null) {
                imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }
            aPressedLast = aNow;

            // Start align on X (reset control points)
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast && imu != null) {
                xImuAlignActive = true;
                alignTimer.reset();
            }
            xPressedLast = xNow;

            if (xImuAlignActive && imu != null) {
                double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double error = imuAlignAngle - imuAngle;
                double turnPower = kP * error;

                frontLeftDrive.setPower(-turnPower);
                backLeftDrive.setPower(-turnPower);
                frontRightDrive.setPower(turnPower);
                backRightDrive.setPower(turnPower);

                telemetry.addData("IMU Angle", imuAngle);
                telemetry.addData("Target Angle", imuAlignAngle);
                telemetry.addData("Error", error);
                telemetry.update();

                if (Math.abs(error) <= angleTolerance || alignTimer.seconds() > alignTimeout) {
                    frontLeftDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backRightDrive.setPower(0);
                    xImuAlignActive = false;
                }
                continue;
            }

            // DRIVE
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

            // SHOOTER CONTROLS
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

            // INTAKE
            if (!gateController.isBusy()) {
                if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                    intakeMotor.setDirection(DcMotor.Direction.REVERSE);
                    intakeMotor.setPower(1.0);
                } else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                    intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                    intakeMotor.setPower(1.0);
                } else {
                    intakeMotor.setPower(0.0);
                }
            }

            // GATE TOGGLE
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && !gateController.isBusy()) {
                gateController.toggleGate();
            }
            bPressedLast = bNow;

            // INTAKE AUTO SEQUENCE
            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast && !gateController.isBusy()) {
                gateController.startIntakeSequence(nowMs);
            }
            yPressedLast = yNow;

            // Gate/claw update
            boolean shouldTriggerClaw = gateController.update(nowMs);
            if (shouldTriggerClaw) clawController.trigger(nowMs);

            // Flywheel update
            flywheel.setTargetRpm(targetRPM);
            flywheel.update();

            boolean atTargetNow = flywheel.isAtTarget();
            if (atTargetNow && !atTargetLast) {
                gamepad1.rumble((int) RUMBLE_DURATION_MS);
                gamepad2.rumble((int) RUMBLE_DURATION_MS);
            }
            atTargetLast = atTargetNow;

            telemetry.addData("Gate Pos", gateServo.getPosition());
            telemetry.addData("RPM", "%.1f", flywheel.getCurrentRPM());
            telemetry.addData("Target RPM", "%.1f", flywheel.getTargetRpm());
            telemetry.addData("Shooter Power", "%.3f", flywheel.getLastAppliedPower());
            telemetry.addData("PIDF Mode", flywheel.isUsingFarCoefficients() ? "FAR" : "CLOSE");
            telemetry.addData("IMU Reference Angle", imuAlignAngle);
            telemetry.update();
        }
    }
}