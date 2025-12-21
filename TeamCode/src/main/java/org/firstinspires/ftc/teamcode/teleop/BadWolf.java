package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="BadWolf", group="Linear OpMode")
public class BadWolf extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, intakeMotor;
    private Servo clawServo=null;
    private Servo leftHoodServo=null;
    private CRServo transferMotor;   // NEW continuous servo

    // shooter control state
    private boolean shooterOn = true;
    private static final double MAX_RPM = 200;
    private static final double TICKS_PER_REV = 537.6;
    private double currentRPM = 0.0;
    private int lastShooterPosition = 0;
    private long lastShooterTime = 0L;

    private double targetRPM = 6000.0;
    private double kP = 0.0003;
    private double emaAlpha = 0.15;

    private double rpmScale = 0.75;
    private boolean xPressedLast = false;
    private boolean yPressedLast = false;
    private int clawActionPhase = 0;
    private long clawActionStartMs = 0L;


    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean dpadUpLast = false;
    private boolean atTargetLast = false;
    private boolean rumbling = false;
    private long rumbleEndTimeMs = 0L;
    private static final double TARGET_TOLERANCE_RPM = 5.0;
    private static final long RUMBLE_DURATION_MS = 1000L;

    // hood/claw timing
    private double hoodPosition = 0;
    private double leftHoodPosition = 0.12;
    private long lastLeftHoodAdjustMs = 0L;
    private static final long HOOD_ADJUST_DEBOUNCE_MS = 120L;
    private static final long CLAW_CLOSE_MS = 500L;

    // Speed updating
    // precision driving / macro buttons
    private double driveScale = 1.0;              // current drive scale (1.0 = normal)
    private static final double PRECISION_SCALE = 0.25; // slow precision multiplier
    private boolean leftBumperLast = false;
    private boolean rightBumperLast = false;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        leftHoodServo = hardwareMap.get(Servo.class, "hoodServo");
        transferMotor = hardwareMap.get(CRServo.class, "transfer"); // NEW mapping
        transferMotor.setDirection(CRServo.Direction.FORWARD);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lastShooterPosition = shooter.getCurrentPosition();
        lastShooterTime = System.currentTimeMillis();
        targetRPM = 120;
        shooterOn = false;
        clawServo.setPosition(0.0);

        // initial positions
        leftHoodServo.setPosition(leftHoodPosition);

        waitForStart();

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // DRIVE
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Macro buttons: left bumper -> precision/slow mode, right bumper -> normal mode
            boolean leftBumperNow = gamepad1.left_bumper || gamepad2.left_bumper;
            if (leftBumperNow && !leftBumperLast) {
                driveScale = PRECISION_SCALE;
            }
            leftBumperLast = leftBumperNow;

            boolean rightBumperNow = gamepad1.right_bumper || gamepad2.right_bumper;
            if (rightBumperNow && !rightBumperLast) {
                driveScale = 1.0;
            }
            rightBumperLast = rightBumperNow;

            frontLeftDrive.setPower(frontLeftPower * driveScale);
            backLeftDrive.setPower(backLeftPower * driveScale);
            frontRightDrive.setPower(frontRightPower * driveScale);
            backRightDrive.setPower(backRightPower * driveScale);

            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) {
                shooter.setDirection(DcMotor.Direction.FORWARD);
                shooterOn = !shooterOn;
            }
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) targetRPM = Math.max(0.0, targetRPM - 10.0);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) targetRPM = Math.min(MAX_RPM, targetRPM + 10.0);
            dpadRightLast = dpadRightNow;

            boolean dpadUpNow = gamepad1.dpad_up || gamepad2.dpad_up;
            if (dpadUpNow && !dpadUpLast) {
                shooter.setDirection(DcMotor.Direction.REVERSE);
                shooterOn = !shooterOn;
            }
            dpadUpLast = dpadUpNow;

            // INTAKE
            if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                transferMotor.setDirection(DcMotor.Direction.FORWARD);
                intakeMotor.setPower(1.0);
                transferMotor.setPower(1.0);
            } else {
                intakeMotor.setPower(0.0);
                transferMotor.setPower(0.0);
            }

            if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                intakeMotor.setDirection(DcMotor.Direction.REVERSE);
                transferMotor.setDirection(DcMotor.Direction.REVERSE);
                intakeMotor.setPower(1.0);
                transferMotor.setPower(1.0);
            } else {
                intakeMotor.setPower(0.0);
                transferMotor.setPower(0.0);
            }

            // Hood adjustments
            if (gamepad1.a && nowMs - lastLeftHoodAdjustMs >= HOOD_ADJUST_DEBOUNCE_MS) {
                lastLeftHoodAdjustMs = nowMs;
                leftHoodPosition = Math.min(0.45, leftHoodPosition + 0.025);
                leftHoodServo.setPosition(leftHoodPosition);
            }
            if (gamepad1.b && nowMs - lastLeftHoodAdjustMs >= HOOD_ADJUST_DEBOUNCE_MS) {
                lastLeftHoodAdjustMs = nowMs;
                leftHoodPosition = Math.max(0.12, leftHoodPosition - 0.025);
                leftHoodServo.setPosition(leftHoodPosition);
            }

            // RPM measurement
            int currentPosition = shooter.getCurrentPosition();
            int deltaTicks = currentPosition - lastShooterPosition;
            long deltaTimeMs = nowMs - lastShooterTime;
            if (deltaTimeMs <= 0) deltaTimeMs = 1;
            double ticksPerSec = (deltaTicks * 1000.0) / deltaTimeMs;
            double measuredRPMRaw = (ticksPerSec / TICKS_PER_REV) * 60.0;
            double measuredRPMScaled = measuredRPMRaw * rpmScale;

            currentRPM = (1.0 - emaAlpha) * currentRPM + emaAlpha * measuredRPMScaled;
            if (currentRPM < 0.0) currentRPM = 0.0;
            lastShooterPosition = currentPosition;
            lastShooterTime = nowMs;

            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast) {
                double safeMeasured = Math.abs(measuredRPMRaw);
                if (safeMeasured >= 1.0) {
                    double candidateScale = targetRPM / measuredRPMRaw;
                    rpmScale = Math.max(0.2, Math.min(3.0, candidateScale));
                }
            }
            yPressedLast = yNow;

            double ff = targetRPM / Math.max(1.0, MAX_RPM);
            double error = targetRPM - currentRPM;
            double pTerm = kP * error;
            double shooterPower = ff + pTerm;
            shooterPower = Math.max(0.0, Math.min(1.0, shooterPower));
            shooter.setPower(shooterOn ? shooterPower : 0.0);

            boolean atTargetNow = Math.abs(targetRPM - currentRPM) <= TARGET_TOLERANCE_RPM;
            if (atTargetNow && !atTargetLast) {
                rumbling = true;
                rumbleEndTimeMs = nowMs + RUMBLE_DURATION_MS;
                try { gamepad1.rumble((int) RUMBLE_DURATION_MS); } catch (Throwable ignored) {}
                try { gamepad2.rumble((int) RUMBLE_DURATION_MS); } catch (Throwable ignored) {}
            }
            atTargetLast = atTargetNow;
            if (rumbling && nowMs > rumbleEndTimeMs) rumbling = false;

            // CLAW toggle
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) {
                clawServo.setPosition(1.0);
                clawActionPhase = 1;
                clawActionStartMs = nowMs;
            }
            xPressedLast = xNow;
            if (clawActionPhase == 1 && nowMs >= clawActionStartMs + CLAW_CLOSE_MS) {
                clawServo.setPosition(0.0);
                clawActionPhase = 0;
            }

            // TELEMETRY
            telemetry.addData("RPM", "%.1f", currentRPM);
            telemetry.addData("Target RPM", "%.1f", targetRPM);
            telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("Hood Position", "%.2f", hoodPosition);
            telemetry.addData("Claw Position", "%.2f", clawServo.getPosition());
            telemetry.addData("Intake Power", "%.2f", intakeMotor.getPower());
            telemetry.addData("Transfer Power", "%.2f", transferMotor.getPower());
            telemetry.update();
        }
    }
}