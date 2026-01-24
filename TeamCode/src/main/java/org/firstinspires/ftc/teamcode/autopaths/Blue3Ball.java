package org.firstinspires.ftc.teamcode.autopaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * ThreeBallBlueBackAndShoot
 *
 * Sequence (adjusted per request):
 *  - Drive backward farther to a mid/close range position (time-based).
 *  - Turn shooter ON once at the start (open-loop), do not repeatedly set power during sequence.
 *  - Intake direction set to REVERSE, transfer CRServo set to REVERSE.
 *  - Feed 2 balls using transfer + intake (longer run so balls travel up ramp).
 *  - Keep intake & transfer running longer so balls can travel up the ramp.
 *  - With intake & transfer still running, open claw (starts CLOSED) to push final ball into the shooter.
 *  - Turn shooter OFF at the end.
 *
 * Hardware names must match BadWolf teleop:
 *  - frontLeft, backLeft, frontRight, backRight (drive motors)
 *  - shooter (DcMotor)
 *  - intakeMotor (DcMotor)
 *  - transfer (CRServo)
 *  - clawServo (Servo)
 *
 * NOTE: This routine is time-based (no sensors). Tune timings/powers on your robot.
 */
@Autonomous(name = "Blue3Ball", group = "Autonomous")
public class Blue3Ball extends LinearOpMode {

    // Drive motors (from BadWolf teleop mapping)
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;

    // Shooter / feeder hardware
    private DcMotor shooter = null;
    private DcMotor intakeMotor = null;
    private CRServo transfer = null;
    private Servo clawServo = null;

    // Shooter RPM estimation (encoder ticks -> RPM) - kept for telemetry only
    private static final double TICKS_PER_REV = 28.0;
    private double currentRPM = 0.0;
    private int lastShooterPosition = 0;
    private long lastShooterTimeMs = 0L;

    // Controller tuning (not used to repeatedly set power anymore)
    private double targetRPM = 120.0;    // informational
    private double emaAlpha = 0.20;      // smoothing for telemetry RPM only

    // Timings / distances (ms) - increased for farther backing and longer feed
    private static final long DRIVE_BACK_MS = 3000;       // increased to back up farther
    private static final double DRIVE_POWER = 0.35   ;
    private static final long SHOOTER_SPINUP_MS = 1200L;  // open-loop spin-up wait at start
    private static final long FEED_DURATION_MS = 1200L;   // increased so balls can travel up the ramp
    private static final long BETWEEN_FEEDS_MS = 500L;    // increased pause between feeds
    private static final long CLAW_OPEN_MS = 600L;        // hold open longer to push ball in

    // Intake / transfer / claw constants
    private static final double INTAKE_POWER = 1.0;
    private static final double INTAKE_OFF = 0.0;
    private static final double TRANSFER_ON = 1.0;
    private static final double TRANSFER_OFF = 0.0;
    private static final double CLAW_CLOSED_POS = 0.0; // start CLOSED
    private static final double CLAW_OPEN_POS = 1.0;   // open to push ball

    // Shooter open-loop power (applied once at start)
    private static final double SHOOTER_APPLIED_POWER = 0.65; // tune this to reach ~120 RPM on your robot

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Init", "Mapping hardware...");
        telemetry.update();

        // Drive mapping (match BadWolf)
        try {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
            frontRight= hardwareMap.get(DcMotor.class, "frontRight");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("HW", "Drive mapping failed: " + e.getMessage());
        }

        // Shooter & feeder mapping
        try {
            shooter = hardwareMap.get(DcMotor.class, "shooter");
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastShooterPosition = shooter.getCurrentPosition();
            lastShooterTimeMs = System.currentTimeMillis();
        } catch (Exception e) {
            telemetry.addData("HW", "Shooter mapping failed: " + e.getMessage());
        }

        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            // Set intake direction to REVERSE (as requested)
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setPower(INTAKE_OFF);
        } catch (Exception e) {
            telemetry.addData("HW", "Intake mapping failed: " + e.getMessage());
        }

        try {
            transfer = hardwareMap.get(CRServo.class, "transfer");
            // Set transfer CRServo to REVERSE (as requested)
            transfer.setDirection(CRServo.Direction.REVERSE);
            transfer.setPower(TRANSFER_OFF);
        } catch (Exception e) {
            telemetry.addData("HW", "Transfer mapping failed: " + e.getMessage());
        }

        try {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            if (clawServo != null) clawServo.setPosition(CLAW_CLOSED_POS); // start CLOSED as requested
        } catch (Exception e) {
            telemetry.addData("HW", "Claw mapping failed: " + e.getMessage());
        }

        telemetry.addData("Init", "Ready");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // 1) Drive backward farther to mid/close position (time-based)
        telemetry.addData("Drive", "Backing up to range (farther)...");
        telemetry.update();
        setDrivePower(-DRIVE_POWER, -DRIVE_POWER, -DRIVE_POWER, -DRIVE_POWER); // negative for backward
        long driveStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - driveStart < DRIVE_BACK_MS) {
            sleep(5);
        }
        stopDrive();
        sleep(150); // brief settle

        // 2) Turn shooter ON ONCE at the start (open-loop)
        telemetry.addData("Shooter", "Turning on (open-loop) with power=%.2f", SHOOTER_APPLIED_POWER);
        telemetry.update();
        if (shooter != null) shooter.setPower(SHOOTER_APPLIED_POWER);

        // Wait a short amount for shooter to spin up (open-loop)
        long spinStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - spinStart < SHOOTER_SPINUP_MS) {
            // update telemetry RPM estimate (optional)
            updateShooterRPM(System.currentTimeMillis());
            telemetry.addData("Shooter RPM (est)", "%.1f", currentRPM);
            telemetry.update();
            sleep(20);
        }

        // 3) Feed 2 balls (transfer + intake pulses) — longer feed duration so balls can travel up ramp
        telemetry.addData("Feed", "Feeding 2 balls (longer run)");
        telemetry.update();
        for (int i = 0; i < 2 && opModeIsActive(); i++) {
            if (transfer != null) transfer.setPower(TRANSFER_ON);
            if (intakeMotor != null) intakeMotor.setPower(INTAKE_POWER);

            long feedStart = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - feedStart < FEED_DURATION_MS) {
                // optional telemetry RPM estimate
                updateShooterRPM(System.currentTimeMillis());
                telemetry.addData("Feed", "ball %d time=%.1fs rpm=%.1f", i+1, (System.currentTimeMillis()-feedStart)/1000.0, currentRPM);
                telemetry.update();
                sleep(10);
            }

            // stop transfer/intake between shots
            if (transfer != null) transfer.setPower(TRANSFER_OFF);
            if (intakeMotor != null) intakeMotor.setPower(INTAKE_OFF);

            long betweenStart = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - betweenStart < BETWEEN_FEEDS_MS) {
                updateShooterRPM(System.currentTimeMillis());
                telemetry.addData("Between", "waiting rpm=%.1f", currentRPM);
                telemetry.update();
                sleep(10);
            }
        }

        // 4) Final ball: keep intake & transfer ON for longer, then open claw (starts CLOSED) to push ball in
        telemetry.addData("Final", "Using claw to push final ball while intake+transfer remain on (longer)");
        telemetry.update();

        if (transfer != null) transfer.setPower(TRANSFER_ON);
        if (intakeMotor != null) intakeMotor.setPower(INTAKE_POWER);

        // ensure claw is closed first
        if (clawServo != null) clawServo.setPosition(CLAW_CLOSED_POS);
        sleep(100); // settle

        // open claw to push the ball
        if (clawServo != null) clawServo.setPosition(CLAW_OPEN_POS);
        long clawOpenStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - clawOpenStart < CLAW_OPEN_MS) {
            updateShooterRPM(System.currentTimeMillis());
            telemetry.addData("ClawPush", "time=%.1fs rpm=%.1f", (System.currentTimeMillis()-clawOpenStart)/1000.0, currentRPM);
            telemetry.update();
            sleep(10);
        }

        // stop feeders and shooter and set claw to closed/resting position
        if (transfer != null) transfer.setPower(TRANSFER_OFF);
        if (intakeMotor != null) intakeMotor.setPower(INTAKE_OFF);
        if (shooter != null) shooter.setPower(0.0); // turn off shooter once at end
        if (clawServo != null) clawServo.setPosition(CLAW_CLOSED_POS);

        telemetry.addData("Auto", "Sequence complete");
        telemetry.update();

        sleep(250);

        // Time-based rotate ~120 degrees then drive forward 2 seconds (approximate)
        final double ROTATE_POWER = 0.5;    // power used for rotation (tune)
        final long ROTATE_120_MS = 850L;    // estimated ms to rotate 120 degrees (tune for your bot)
        final long FORWARD_MS = 2000L;      // 2 seconds forward

        /*// rotate in place left (change for diff alliance)
        setDrivePower(-DRIVE_POWER, -DRIVE_POWER, DRIVE_POWER, DRIVE_POWER);
        long driveEnd1 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - driveEnd1 < 1000) {
            sleep(5);
        }
        setDrivePower(0, 0, 0, 0);

        // move forward for 2 seconds
        setDrivePower(DRIVE_POWER, DRIVE_POWER, DRIVE_POWER, DRIVE_POWER);
        long driveEnd2 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - driveEnd2 < FORWARD_MS) {
            sleep(5);
        }
        setDrivePower(0, 0, 0, 0);*/


        setDrivePower(-DRIVE_POWER, DRIVE_POWER, DRIVE_POWER, -DRIVE_POWER);
        long driveEnd1 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - driveEnd1 < 1500) {
            sleep(5);
        }
        setDrivePower(0, 0, 0, 0);
    }


    // Utility: set four-drive motor powers (LF, LR, RF, RR)
    private void setDrivePower(double fl, double bl, double fr, double br) {
        try {
            if (frontLeft != null) frontLeft.setPower(fl);
            if (backLeft != null) backLeft.setPower(bl);
            if (frontRight != null) frontRight.setPower(fr);
            if (backRight != null) backRight.setPower(br);
        } catch (Exception ignored) {}
    }

    private void stopDrive() {
        setDrivePower(0,0,0,0);
    }

    /**
     * Estimate shooter RPM from encoder ticks and smooth via EMA.
     * Updates currentRPM, lastShooterPosition, and lastShooterTimeMs.
     * This is for telemetry only (we no longer change shooter power repeatedly).
     */
    private void updateShooterRPM(long nowMs) {
        if (shooter == null) return;
        try {
            int pos = shooter.getCurrentPosition();
            long dtMs = nowMs - lastShooterTimeMs;
            if (dtMs <= 0) {
                lastShooterTimeMs = nowMs;
                lastShooterPosition = pos;
                return;
            }
            int dTicks = pos - lastShooterPosition;
            double measuredRpm = (dTicks / TICKS_PER_REV) / (dtMs / 1000.0) * 60.0;
            currentRPM = emaAlpha * measuredRpm + (1.0 - emaAlpha) * currentRPM;
            lastShooterPosition = pos;
            lastShooterTimeMs = nowMs;
        } catch (Exception ignored) { }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}