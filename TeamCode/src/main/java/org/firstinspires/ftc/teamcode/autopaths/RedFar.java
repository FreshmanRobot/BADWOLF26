package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;

/**
 * BlueFar Autonomous - 3 Ball Blue Alliance
 *
 * Sequence:
 * 1. Start flywheel at 4000 RPM
 * 2. Turn right 30 degrees using time-based rotation
 * 3. Shoot 3 balls using intake and gate sequence
 * 4. Turn left 30 degrees using time-based rotation
 * 5. Move out of shooting zone
 */
@Autonomous(name = "RedFar", group = "Autonomous")
@Configurable
public class RedFar extends OpMode {

    // Drive motors
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // Shooter motors
    private DcMotorEx shooterEx;
    private DcMotor shooter2;

    // Intake and gate components
    private DcMotor intakeMotor;
    private Servo clawServo;
    private Servo gateServo;
    private Servo leftHoodServo;

    // Controllers
    private FlywheelController flywheel;
    private GateController gateController;
    private ClawController clawController;

    // Constants
    private static final double GATE_OPEN = 0.28;
    private static final double GATE_CLOSED = 0.60;
    private static final double CLAW_OPEN = 0.2;
    private static final double CLAW_CLOSED = 0.63;
    private static final long CLAW_CLOSE_MS = 500L;
    private static final long INTAKE_DURATION_MS = 1200;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 100;
    private static final double INTAKE_SEQUENCE_POWER = 1.0;
    private static final double LEFT_HOOD_POSITION = 0.43;

    // Target RPM for far shot
    private static final double TARGET_RPM = 4000.0;

    // State machine
    private enum AutoState {
        INIT_FLYWHEEL,
        TURN_RIGHT_30,
        SHOOT_SEQUENCE,
        TURN_LEFT_30,
        MOVE_OUT,
        DONE
    }

    private AutoState currentState = AutoState.INIT_FLYWHEEL;
    private long stateStartTime = 0L;
    private int ballsShot = 0;

    @Override
    public void init() {
        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        // Set drive motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize shooter motors
        shooterEx = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

        shooterEx.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize intake and servo components
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        leftHoodServo = hardwareMap.get(Servo.class, "hoodServo");

        // Set initial servo positions
        clawServo.setPosition(CLAW_OPEN);
        leftHoodServo.setPosition(LEFT_HOOD_POSITION);
        gateServo.setPosition(GATE_CLOSED);

        // Initialize controllers
        clawController = new ClawController(clawServo, CLAW_OPEN, CLAW_CLOSED, CLAW_CLOSE_MS);
        gateController = new GateController(
                gateServo, intakeMotor,
                GATE_OPEN, GATE_CLOSED,
                INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                INTAKE_SEQUENCE_POWER
        );

        // Initialize flywheel controller
        VoltageSensor voltageSensor = null;
        try {
            if (hardwareMap.voltageSensor.iterator().hasNext()) {
                voltageSensor = hardwareMap.voltageSensor.iterator().next();
            }
        } catch (Exception ignored) {}

        flywheel = new FlywheelController(shooterEx, shooter2, telemetry, voltageSensor);
        flywheel.setTargetRpm(TARGET_RPM);

        telemetry.addData("Status", "Initialized - 3 Ball Blue Far");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Ready to start - 3 Ball Blue Far");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.update();
    }

    @Override
    public void start() {
        currentState = AutoState.INIT_FLYWHEEL;
        stateStartTime = System.currentTimeMillis();
        ballsShot = 0;
        flywheel.setShooterOn(true);
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();

        // Update flywheel
        flywheel.setTargetRpm(TARGET_RPM);
        flywheel.update();

        // Update gate and claw controllers
        boolean shouldTriggerClaw = gateController.update(nowMs);
        if (shouldTriggerClaw) {
            clawController.trigger(nowMs);
        }
        clawController.update(nowMs);

        // State machine
        switch (currentState) {
            case INIT_FLYWHEEL:
                // Start the flywheel and wait for it to reach target RPM
                if (flywheel.isAtTarget()) {
                    currentState = AutoState.TURN_RIGHT_30;
                    stateStartTime = nowMs;
                    telemetry.addData("State", "Turning left 30 degrees");
                } else {
                    telemetry.addData("State", "Spinning up flywheel");
                    telemetry.addData("Current RPM", "%.1f", flywheel.getCurrentRPM());
                }
                break;

            case TURN_RIGHT_30:
                // Turn left 30 degrees using time-based rotation
                turn30(1, 1, -1, -1);
                currentState = AutoState.SHOOT_SEQUENCE;
                stateStartTime = nowMs;
                telemetry.addData("State", "Starting shoot sequence");
                break;

            case SHOOT_SEQUENCE:
                // Start the intake sequence to shoot all 3 balls
                if (ballsShot < 3 && !gateController.isBusy()) {
                    gateController.startIntakeSequence(nowMs);
                    ballsShot++;
                    telemetry.addData("State", "Shooting ball " + ballsShot + " of 3");
                }

                // Check if all 3 balls have been shot
                if (ballsShot >= 3 && !gateController.isBusy()) {
                    currentState = AutoState.TURN_LEFT_30;
                    flywheel.setShooterOn(false);
                    telemetry.addData("State", "All balls shot - Complete!");
                } else if (!gateController.isBusy()) {
                    telemetry.addData("State", "Waiting for intake sequence");
                }
                break;

            case TURN_LEFT_30:
                // Turn right 30 degrees using time-based rotation
                turn30(-1, -1, 1, 1);
                currentState = AutoState.MOVE_OUT;
                stateStartTime = nowMs;
                telemetry.addData("State", "Starting move out");
                break;

            case MOVE_OUT:
                long t = System.currentTimeMillis();
                while (System.currentTimeMillis() - t < 1000) {
                    frontLeftDrive.setPower(0.4);
                    backLeftDrive.setPower(-0.4);
                    frontRightDrive.setPower(-0.4);
                    backRightDrive.setPower(0.4);
                }
                currentState = AutoState.DONE;
                stateStartTime = nowMs;

            case DONE:
                // Stop all motors
                stopAllMotors();
                telemetry.addData("State", "AUTO COMPLETE - 3 balls shot!");
                break;

            default:
                break;
        }

        // Telemetry
        telemetry.addData("Current State", currentState.name());
        telemetry.addData("Flywheel RPM", "%.1f", flywheel.getCurrentRPM());
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Balls Shot", ballsShot);
        telemetry.addData("Gate Position", gateServo.getPosition());
        telemetry.update();
    }

    /**
     * Turn left 30 degrees using time-based rotation (0.27 seconds at 0.5 power).
     * This uses the same logic as the provided turnLeft30() method.
     */
    private void turn30(int a, int b, int c, int d) {
        double power = 0.4;
        long duration = 270; // milliseconds (0.27 seconds)

        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < duration) {
            // Left side backward, right side forward (counter-clockwise turn)
            frontLeftDrive.setPower(power * a);
            backLeftDrive.setPower(power * b);
            frontRightDrive.setPower(power * c);
            backRightDrive.setPower(power * d);
        }

        // Stop all drive motors
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    /**
     * Stop all motors safely.
     */
    private void stopAllMotors() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        intakeMotor.setPower(0);
        flywheel.setShooterOn(false);
    }


    @Override
    public void stop() {
        stopAllMotors();
    }
}