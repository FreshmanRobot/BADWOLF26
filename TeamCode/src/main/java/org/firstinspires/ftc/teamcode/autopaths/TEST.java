package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;

/**
 * BlueFarAuto - 3 Ball Blue Alliance (Pedro Path Follower)
 *
 * Sequence:
 * 1. Start flywheel at 4000 RPM
 * 2. Follow path to shooting position
 * 3. Shoot 3 balls using intake and gate sequence
 * 4. Move out of shooting zone
 */
@Autonomous(name = "TEST", group = "Autonomous")
@Configurable
public class TEST extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // Heading constants for shooting
    private static final double SHOOT_HEADING_RAD = Math.toRadians(90);
    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(4);

    private enum AutoState { IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, PRE_ACTION, INTAKE_WAIT, CLAW_ACTION, FINISHED }
    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    private Timer intakeTimer;
    private static final double INTAKE_WAIT_SECONDS = 2.2;

    private double targetRPM = 4000.0;

    private long clawActionStartMs = 0L;
    private static final long CLAW_CLOSE_MS = 250L;

    private static final double CLAW_CLOSED = 0.63;
    private static final double CLAW_OPEN = 0.2;

    private Timer preActionTimer;
    private static final double PRE_ACTION_WAIT_SECONDS = 0.3;

    private Timer poseWaitTimer;
    private static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.3;

    private boolean preActionTimerStarted = false;
    private boolean preActionEntered = false;

    private long shooterWaitStartMs = -1;
    private static final long SHOOTER_WAIT_TIMEOUT_MS = 2000L;

    private DcMotorEx shooterMotor;
    private DcMotor shooter2;

    private BNO055IMU imu = null;

    private FlywheelController flywheel;
    private GateController gateController;
    private ClawController clawController;

    private DcMotor intakeMotor;
    private Servo gateServo = null;
    private Servo leftHoodServo = null;
    private static final double LEFT_HOOD_POSITION = 0.43;

    private Servo clawServo;

    private static final double GATE_OPEN = 0.28;
    private static final double GATE_CLOSED = 0.60;

    private static final long INTAKE_DURATION_MS = 800;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 100;
    private static final double INTAKE_SEQUENCE_POWER = 1.0;

    // Shooting pose constants
    private static final double SHOOT_POSE_X = 60.0;
    private static final double SHOOT_POSE_Y = 10.0;
    private static final double START_POSE_X = 60.0;
    private static final double START_POSE_Y = 8.0;
    private static final double START_POSE_HEADING = Math.toRadians(90);
    private static final double END_POSE_X = 10.0;
    private static final double END_POSE_Y = 8.6;
    private static final double END_POSE_HEADING = Math.toRadians(180);
    private static final double START_POSE_TOLERANCE_IN = 6.0;

    private int ballsShot = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        intakeTimer = new Timer();
        preActionTimer = new Timer();
        poseWaitTimer = new Timer();
        nextPathIndex = -1;
        preActionTimerStarted = false;
        preActionEntered = false;
        ballsShot = 0;

        // Map gate servo
        try {
            gateServo = hardwareMap.get(Servo.class, "gateServo");
            gateServo.setPosition(GATE_CLOSED);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Gate servo mapping failed: " + e.getMessage());
        }

        // Shooter motors
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            shooterMotor.setDirection(DcMotor.Direction.FORWARD);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor.setPower(0.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Shooter map fail: " + e.getMessage());
        }

        try {
            shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
            shooter2.setDirection(DcMotor.Direction.REVERSE);
            shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter2.setPower(0.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Shooter2 map fail: " + e.getMessage());
        }

        // Intake
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setPower(0.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Intake mapping failed: " + e.getMessage());
        }

        // Claw
        try {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            if (clawServo != null) clawServo.setPosition(CLAW_OPEN);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Claw servo mapping failed: " + e.getMessage());
        }

        // Hood
        try {
            leftHoodServo = hardwareMap.get(Servo.class, "hoodServo");
            if (leftHoodServo != null) leftHoodServo.setPosition(LEFT_HOOD_POSITION);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Hood map fail: " + e.getMessage());
        }

        // Voltage sensor
        VoltageSensor voltageSensor = null;
        try {
            if (hardwareMap.voltageSensor.iterator().hasNext()) {
                voltageSensor = hardwareMap.voltageSensor.iterator().next();
            }
        } catch (Exception ignored) {}

        // Controllers
        flywheel = new FlywheelController(shooterMotor, shooter2, telemetry, voltageSensor);
        flywheel.setTargetRpm(targetRPM);

        if (clawServo != null) {
            clawController = new ClawController(clawServo, CLAW_OPEN, CLAW_CLOSED, CLAW_CLOSE_MS);
        }
        if (gateServo != null && intakeMotor != null) {
            gateController = new GateController(
                    gateServo, intakeMotor,
                    GATE_OPEN, GATE_CLOSED,
                    INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                    INTAKE_SEQUENCE_POWER
            );
            gateServo.setPosition(GATE_CLOSED);
        }

        panelsTelemetry.debug("Status", "Initialized - 3 Ball Blue Far");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        follower.setStartingPose(new Pose(START_POSE_X, START_POSE_Y, START_POSE_HEADING));
    }

    @Override
    public void start() {
        flywheel.setTargetRpm(targetRPM);
        flywheel.setShooterOn(true);
        startIntake();
        follower.setStartingPose(new Pose(START_POSE_X, START_POSE_Y, START_POSE_HEADING));

        shooterWaitStartMs = System.currentTimeMillis();
        state = AutoState.WAIT_FOR_SHOOTER;
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();

        flywheel.setTargetRpm(targetRPM);
        flywheel.update();

        follower.update();

        runStateMachine(nowMs);

        if (gateController != null) {
            gateController.update(nowMs);
        }

        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("PathIdx", currentPathIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("DistToShootPose", String.format("%.2f", distanceToShootPose()));
        panelsTelemetry.debug("Balls Shot", ballsShot);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setPower(0.0);
        if (shooter2 != null) shooter2.setPower(0.0);
        stopIntake();
        if (clawServo != null) clawServo.setPosition(CLAW_OPEN);
        if (gateServo != null) gateServo.setPosition(GATE_CLOSED);
        state = AutoState.FINISHED;
    }

    private void startIntake() {
        try {
            if (intakeMotor != null) intakeMotor.setPower(1.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Intake", "startIntake error: " + e.getMessage());
        }
    }

    private void stopIntake() {
        try {
            if (intakeMotor != null) intakeMotor.setPower(0.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Intake", "stopIntake error: " + e.getMessage());
        }
    }

    private double distanceToShootPose() {
        try {
            Pose p = follower.getPose();
            double dx = p.getX() - SHOOT_POSE_X;
            double dy = p.getY() - SHOOT_POSE_Y;
            return Math.hypot(dx, dy);
        } catch (Exception e) {
            return Double.POSITIVE_INFINITY;
        }
    }

    private void startPath(int idx) {
        if (idx < 1 || idx > 3) {
            currentPathIndex = 0;
            state = AutoState.FINISHED;
            return;
        }

        switch (idx) {
            case 1:
                follower.followPath(paths.StartToShoot);
                startIntake();
                break;
            case 2:
                follower.followPath(paths.ShootToEnd);
                stopIntake();
                break;
            case 3:
                state = AutoState.FINISHED;
                break;
            default:
                break;
        }

        currentPathIndex = idx;
        state = AutoState.RUNNING_PATH;
    }

    private void runStateMachine(long nowMs) {
        switch (state) {
            case WAIT_FOR_SHOOTER: {
                long elapsed = (shooterWaitStartMs < 0) ? 0 : (nowMs - shooterWaitStartMs);
                if (elapsed >= SHOOTER_WAIT_TIMEOUT_MS || (flywheel != null && flywheel.isAtTarget())) {
                    startPath(1);
                }
                break;
            }

            case RUNNING_PATH: {
                if (!follower.isBusy()) {
                    int finished = currentPathIndex;

                    if (finished == 1) {
                        // Reached shooting position
                        nextPathIndex = 2;
                        preActionTimerStarted = false;
                        preActionEntered = false;
                        state = AutoState.PRE_ACTION;
                    } else {
                        state = AutoState.FINISHED;
                    }
                }
                break;
            }

            case PRE_ACTION: {
                if (!preActionEntered) {
                    poseWaitTimer.resetTimer();
                    preActionTimerStarted = false;
                    preActionEntered = true;
                    panelsTelemetry.debug("PRE_ACTION", "Entered PRE_ACTION, starting pose/heading wait");
                }

                if (!preActionTimerStarted) {
                    double dist = distanceToShootPose();
                    double targetHeading = SHOOT_HEADING_RAD;

                    double headingErr = headingErrorRad(
                            follower.getPose().getHeading(),
                            targetHeading
                    );

                    boolean poseGood = dist <= START_POSE_TOLERANCE_IN && Math.abs(headingErr) <= HEADING_TOLERANCE_RAD;
                    boolean poseTimeout = poseWaitTimer.getElapsedTimeSeconds() >= PRE_ACTION_MAX_POSE_WAIT_SECONDS;

                    if (poseGood || poseTimeout) {
                        try {
                            follower.setPose(new Pose(SHOOT_POSE_X, SHOOT_POSE_Y, SHOOT_HEADING_RAD));
                        } catch (Exception ignored) {}

                        preActionTimer.resetTimer();
                        preActionTimerStarted = true;
                        panelsTelemetry.debug("PRE_ACTION",
                                poseGood ? "At pose+heading: starting timer"
                                        : "Pose timeout: snapping pose+heading and starting timer");
                    } else {
                        panelsTelemetry.debug("PRE_ACTION",
                                String.format("Waiting pose/heading (dist=%.2f, hdgErr=%.2f°)",
                                        dist, Math.toDegrees(headingErr)));
                    }
                } else {
                    if (preActionTimer.getElapsedTimeSeconds() >= PRE_ACTION_WAIT_SECONDS) {
                        intakeTimer.resetTimer();
                        state = AutoState.INTAKE_WAIT;
                    }
                }
                break;
            }

            case INTAKE_WAIT: {
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_SECONDS) {
                    state = AutoState.CLAW_ACTION;
                }
                break;
            }

            case CLAW_ACTION: {
                if (clawActionStartMs == 0) {
                    clawActionStartMs = nowMs;
                    clawServo.setPosition(CLAW_CLOSED);
                }

                if (nowMs - clawActionStartMs >= 207) {
                    clawServo.setPosition(CLAW_OPEN);
                    clawActionStartMs = 0;
                    ballsShot++;

                    if (ballsShot < 3) {
                        // Continue shooting
                        intakeTimer.resetTimer();
                        state = AutoState.INTAKE_WAIT;
                    } else {
                        // All balls shot, move out
                        if (gateServo != null) {
                            gateServo.setPosition(GATE_CLOSED);
                        }
                        startPath(2);
                    }
                }
                break;
            }

            case FINISHED:
            case IDLE:
            default:
                break;
        }
    }

    private double headingErrorRad(double current, double target) {
        double err = target - current;
        while (err > Math.PI) err -= 2 * Math.PI;
        while (err < -Math.PI) err += 2 * Math.PI;
        return err;
    }

    public static class Paths {
        public PathChain StartToShoot;
        public PathChain ShootToEnd;

        public Paths(Follower follower) {
            // Path 1: start(60, 8) to shoot(60, 10) with heading 90° to 110°
            StartToShoot = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 8.000), new Pose(60.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                    .build();

            // Path 2: shoot(60, 10) to end(10, 8.6) with heading 110° to 180°
            ShootToEnd = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000), new Pose(10.000, 8.600)))
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();
        }
    }
}