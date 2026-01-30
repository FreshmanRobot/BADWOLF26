package org.firstinspires.ftc.teamcode.autopaths;

import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;


/**
 * ExperimentalPedroAuto (improved PRE_ACTION pose handling + fallback)
 *
 * Fixes:
 *  - PRE_ACTION timer only begins after the robot reaches the shoot pose OR after a short pose-wait timeout.
 *  - Increased default pose tolerance and added debug telemetry for distance-to-target so you can tune.
 *  - Prevents the robot from stalling indefinitely when it gets "very close but not exact".
 *
 * Behavior preserved otherwise.
 *
 * Modifications:
 *  - When path 4, path 7 or path 10 starts, run intake for a timed 1.0s and then stop.
 *  - PRE_ACTION_WAIT_SECONDS reduced by 0.5s (from 0.8 to 0.3).
 */
@Autonomous(name = "A BW BLUE 12 BALL AUTO", group = "Autonomous")
@Configurable
public class BWBlueAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    //for heading convergence
    private static final double SHOOT_HEADING_RAD = Math.toRadians(130);
    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(3); // 2–4° is good

    // State machine
    private enum AutoState { IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, PRE_ACTION, INTAKE_WAIT, CLAW_ACTION, FINISHED }
    private AutoState state = AutoState.IDLE;

    private boolean preActionIsForShooting = false;

    // current path index being run (1..11). 0 when none.
    private int currentPathIndex = 0;
    // next path index to run after PRE_ACTION/CLAW sequences
    private int nextPathIndex = -1;

    // Pending path start index if a start request happens while follower is busy
    private int pendingStartIndex = -1;

    // Intake-wait state & timer
    private Timer intakeTimer;
    private static final double INTAKE_WAIT_SECONDS = 2.2; // change to shorten/lengthen intake duration

    // Timed intake on-path start (for path4, path7 and path10)
    private Timer timedIntakeTimer;
    private static final double TIMED_INTAKE_SECONDS = 0.7; // run intake for 1 second when path4, path7 or path10 starts
    private boolean timedIntakeActive = false;

    private double targetRPM = 3000;

    // Claw action state & timing (and positions)
    private long clawActionStartMs = 0L;
    private static final long CLAW_CLOSE_MS = 250L; // duration to hold claw closed

    private static final double CLAW_CLOSED = 0.63; // start CLOSED
    private static final double CLAW_OPEN = 0.2;   // open to push ball
    // Pre-action (delay before starting intake/claw) timer
    private Timer preActionTimer;
    private static final double PRE_ACTION_WAIT_SECONDS = 0.3; // lowered by 0.5s from 0.8

    // Pose-wait timer (wait for robot to reach pose before starting PRE_ACTION). Fallback if it never quite reaches it.
    private Timer poseWaitTimer;
    private static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.3; // fallback after short timeout

    // Flag to indicate whether PRE_ACTION timer has been started (we only start it when robot reaches pose or fallback triggers)
    private boolean preActionTimerStarted = false;
    // Flag set when entering PRE_ACTION state to reset poseWaitTimer
    private boolean preActionEntered = false;

    // Shooter-wait state before starting paths
    private long shooterWaitStartMs = -1;
    private static final long SHOOTER_WAIT_TIMEOUT_MS = 4000L; // fallback timeout if shooter doesn't spin up

    // Shooter / Turret hardware & controllers
    private DcMotor shooterMotor, shooter2;
    private boolean shooterOn = false;

    FlywheelController flywheel;
    GateController gateController;
    ClawController clawController;


    private long lastShooterPosition = 0;
    private long lastShooterTime = 0;

    // Intake + compression hardware (from teleop)
    private DcMotor intakeMotor;
    private CRServo transferMotor;   // NEW continuous servo
    private Servo gateServo = null;
    private Servo leftHoodServo = null;
    private static final double LEFT_HOOD_POSITION = 0.9;


    //wheel hardware
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // Claw servo
    private Servo clawServo;

    // Gate distance-based control (like BluePedroAuto)
    private static final double GATE_OPEN_TOLERANCE_IN = 9.0;
    private static final double GATE_CLOSE_TOLERANCE_IN = 10.0;
    private boolean gateClosed = true;

    // Intake/gate "on" values (match teleop right-trigger behavior)
    private static final double INTAKE_ON_POWER = 1.0;
    private static final double GATE_OPEN = 0.28;
    private static final double GATE_CLOSED = 0.60;

    private static final long INTAKE_DURATION_MS = 1200;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 100;
    private static final double INTAKE_SEQUENCE_POWER = 1.0;
    private int intakeSegmentEnd = -1;

    // Shoot pose constants and tolerance - used to ensure PRE_ACTION timer starts only when robot reaches pose
    private static final double SHOOT_POSE_X = 48.0;
    private static final double SHOOT_POSE_Y = 96.0;
    private static final double START_POSE_TOLERANCE_IN = 6.0; // increased tolerance to avoid tiny-miss stalls


    public BWBlueAuto() {}

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        // Create follower and build paths
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);


        // Timers & state
        intakeTimer = new Timer();
        timedIntakeTimer = new Timer();
        preActionTimer = new Timer();
        poseWaitTimer = new Timer();
        nextPathIndex = -1;
        intakeSegmentEnd = -1;
        preActionTimerStarted = false;
        preActionEntered = false;
        timedIntakeActive = false;
        pendingStartIndex = -1;

        //wheel hardware
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            shooterMotor.setDirection(DcMotor.Direction.FORWARD);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor.setPower(0.0);

            shooterOn = false;
            lastShooterPosition = shooterMotor.getCurrentPosition();
            lastShooterTime = System.currentTimeMillis();
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Shooter map fail: " + e.getMessage());
        }

        try {
            shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
            shooter2.setDirection(DcMotor.Direction.REVERSE);
            shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter2.setPower(0.0);

        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Shooter map fail: " + e.getMessage());
        }

        // --- Intake & compression hardware (same names as teleop) ---
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

            // Direction per request
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);

            gateServo = hardwareMap.get(Servo.class, "gateServo");
            // Set defaults (same as teleop off state)
            intakeMotor.setPower(0.0);


            gateServo.setPosition(GATE_CLOSED);

        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Intake/compression mapping failed: " + e.getMessage());
        }

        // --- Claw servo ---
        try {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            // ensure default open position as teleop uses 0.63 for open
            clawServo.setPosition(CLAW_OPEN);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Claw servo mapping failed: " + e.getMessage());
        }

        try {
            leftHoodServo = hardwareMap.get(Servo.class, "hoodServo");
            if (leftHoodServo != null) leftHoodServo.setPosition(LEFT_HOOD_POSITION);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Hood map fail: " + e.getMessage());
        }

        gateServo = hardwareMap.get(Servo.class, "gateServo");

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        VoltageSensor voltageSensor = null;
        try {
            if (hardwareMap.voltageSensor.iterator().hasNext()) {
                voltageSensor = hardwareMap.voltageSensor.iterator().next();
            }
        } catch (Exception ignored) {}

        // Create FlywheelController: primary (DcMotorEx), secondary (DcMotor), telemetry, voltageSensor (may be null)
        flywheel = new FlywheelController(shooterMotor, shooter2, telemetry, voltageSensor);
        flywheel.setTargetRpm(targetRPM);
        flywheel.setShooterOn(true);

        clawController = new ClawController(clawServo, CLAW_OPEN, CLAW_CLOSED, CLAW_CLOSE_MS);
        gateController = new GateController(
                gateServo, intakeMotor,
                GATE_OPEN, GATE_CLOSED,
                INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                INTAKE_SEQUENCE_POWER
        );

        // Initialize at the Open position
        gateServo.setPosition(GATE_CLOSED);
    }


    @Override
    public void init_loop() {
        follower.setStartingPose(new Pose(20, 122, Math.toRadians(135)));

    }

    @Override
    public void start() {
        flywheel.setTargetRpm(targetRPM);
        // Set starting pose to the start point of Path1 (and heading to match interpolation start)
        follower.setStartingPose(new Pose(20, 122, Math.toRadians(135)));

        // Start spinner and turret references
        if (shooterMotor != null) {
            flywheel.setShooterOn(true);
            shooterOn = true;
            shooterWaitStartMs = System.currentTimeMillis();
        }
        else{
            shooterWaitStartMs = System.currentTimeMillis();
        }

        intakeMotor.setPower(0.5);

        // Start measuring shooter-wait
        shooterWaitStartMs = System.currentTimeMillis();
        state = AutoState.WAIT_FOR_SHOOTER;
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();
        follower.update();
        updateGate();

        //flywheel update
        flywheel.setTargetRpm(targetRPM);
        flywheel.update();

        boolean shouldTriggerClaw = gateController.update(nowMs);
        if (shouldTriggerClaw) clawController.trigger(nowMs);
        clawController.update(nowMs);

        // Run the refined FSM
        runStateMachine(nowMs);

        // Panels & driver station telemetry (including distance-to-target for tuning)
        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("PathIdx", currentPathIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        if (intakeMotor != null) {
            panelsTelemetry.debug("Intake Power", intakeMotor.getPower());
            //panelsTelemetry.debug("LeftCompPos", leftCompressionServo != null ? leftCompressionServo.getPosition() : -1);
            //panelsTelemetry.debug("RightCompPos", rightCompressionServo != null ? rightCompressionServo.getPosition() : -1);
            //panelsTelemetry.debug("Transfer Power?", transferMotor != null ? transferMotor.getPower() : -1);
        }
        if (clawServo != null) {
            panelsTelemetry.debug("ClawPos", clawServo.getPosition());
        }

        double dist = distanceToShootPose();
        panelsTelemetry.debug("DistToShootPose", String.format("%.2f", dist));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        if (shooterMotor != null)shooterMotor.setPower(0.0);
        if (shooter2 != null)shooter2.setPower(0.0);
        // Ensure intake off and claw open
        intakeMotor.setPower(0.0);
        if (clawServo != null) clawServo.setPosition(CLAW_OPEN);
        state = AutoState.FINISHED;
    }

    // --- Intake helpers (right-trigger behavior) ---
    private void startIntake() {
        try {
            if (intakeMotor != null) intakeMotor.setPower(INTAKE_ON_POWER);

        } catch (Exception e) {
            panelsTelemetry.debug("Intake", "startIntake error: " + e.getMessage());
        }
    }

    private void stopIntake() {
        try {
            if (intakeMotor != null) intakeMotor.setPower(0.5);
            //if (leftCompressionServo != null) leftCompressionServo.setPosition(LEFT_COMPRESSION_OFF);
            //if (rightCompressionServo != null) rightCompressionServo.setPosition(RIGHT_COMPRESSION_OFF);
        } catch (Exception e) {
            panelsTelemetry.debug("Intake", "stopIntake error: " + e.getMessage());
        }
    }

    // Helper to check which path indices end at the shoot pose (48,96)
    private boolean endsAtShoot(int pathIndex) {
        return pathIndex == 1 || pathIndex == 4 || pathIndex == 7 || pathIndex == 10;
    }

    /**
     * Returns true when follower's current pose is within tolerance (inches) of target.
     */
    private boolean isAtPose(double targetX, double targetY, double tolerance) {
        try {
            Pose p = follower.getPose();
            double dx = p.getX() - targetX;
            double dy = p.getY() - targetY;
            return Math.hypot(dx, dy) <= tolerance;
        } catch (Exception e) {
            panelsTelemetry.debug("isAtPose", "error: " + e.getMessage());
            return false;
        }
    }

    /**
     * Compute Euclidean distance to shoot pose for telemetry/tuning.
     */
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

    private double distanceToPathEnd() {
        Pose end = getManualPathEnd(currentPathIndex);
        if (end == null) return Double.POSITIVE_INFINITY;

        Pose p = follower.getPose();
        double dx = p.getX() - end.getX();
        double dy = p.getY() - end.getY();

        return Math.hypot(dx, dy);
    }



    /**
     * Start a path by index and set the state to RUNNING_PATH.
     * This helper also starts intake/compression before specific path segments (3, 6, 9).
     * Intake will run only DURING those starting paths and will be stopped when that path finishes,
     * so it will not be active during paths 4, 7, 10 by default.
     *
     * Additionally: when starting path 4, path 7 or path 10 we start a timed intake that runs for 1s and then stops.
     */
    private void startPath(int idx) {
        if (idx < 1 || idx > 11) {
            currentPathIndex = 0;
            state = AutoState.FINISHED;
            return;
        }

        // If follower is busy, queue the path and exit
        if (follower != null && follower.isBusy()) {
            pendingStartIndex = idx;
            panelsTelemetry.debug("StartPath", "Follower busy → queued path " + idx);
            return;
        }

        // Intake segments
        if (idx == 3 || idx == 6 || idx == 9) {
            intakeSegmentEnd = idx;
            startIntake();
        }

        // Timed intake segments
        if (idx == 4 || idx == 7 || idx == 10) {
            startIntake();
            timedIntakeTimer.resetTimer();
            timedIntakeActive = true;
        }

        // Actually start the path
        switch (idx) {
            case 1: follower.followPath(paths.Path1); break;
            case 2: follower.followPath(paths.Path2); break;
            case 3: follower.followPath(paths.Path3); break;
            case 4: follower.followPath(paths.Path4); break;
            case 5: follower.followPath(paths.Path5); break;
            case 6: follower.followPath(paths.Path6); break;
            case 7: follower.followPath(paths.Path7); break;
            case 8: follower.followPath(paths.Path8); break;
            case 9: follower.followPath(paths.Path9); break;
            case 10: follower.followPath(paths.Path10); break;
            case 11: follower.followPath(paths.Path11); break;
        }

        currentPathIndex = idx;
        state = AutoState.RUNNING_PATH;
        panelsTelemetry.debug("StartPath", "Started path " + idx);
    }


    private void runStateMachine(long nowMs) {
        // Handle timed intake expiration independent of the FSM states:
        if (timedIntakeActive) {
            if (timedIntakeTimer.getElapsedTimeSeconds() >= TIMED_INTAKE_SECONDS) {
                // stop the timed intake
                stopIntake();
                timedIntakeActive = false;
                // ensure intakeSegment tracking is cleared so we don't try to stop it again later
                intakeSegmentEnd = -1;
                panelsTelemetry.debug("TimedIntake", "Timed intake ended after " + TIMED_INTAKE_SECONDS + "s");
            } else {
                // still within timed intake period - show telemetry
                panelsTelemetry.debug("TimedIntake", String.format("remaining=%.2fs", TIMED_INTAKE_SECONDS - timedIntakeTimer.getElapsedTimeSeconds()));
            }
        }

        // If we previously scheduled a path because the follower was busy, try to start it now when follower is free
        // If a path is queued, start it as soon as follower is free
        if (pendingStartIndex > 0) {
            if (!follower.isBusy()) {
                int idx = pendingStartIndex;
                pendingStartIndex = -1;
                panelsTelemetry.debug("Scheduler", "Starting queued path " + idx);
                startPath(idx);
                return; // prevent double-processing this loop
            }
        }


        switch (state) {
            case WAIT_FOR_SHOOTER:
                long elapsed = (shooterWaitStartMs < 0) ? 0 : (System.currentTimeMillis() - shooterWaitStartMs);
                if (elapsed >= SHOOTER_WAIT_TIMEOUT_MS) {
                    // proceed to first path immediately
                    startPath(1);
                }
                break;

            case RUNNING_PATH:
                // Only finish the path if BOTH:
                // 1. follower is not busy
                // 2. robot is actually close to the target
                if (!follower.isBusy() || distanceToPathEnd() < 3.0) {

                    int finished = currentPathIndex;

                    // Stop intake if this was an intake-only path
                    if (intakeSegmentEnd == finished) {
                        stopIntake();
                        intakeSegmentEnd = -1;
                    }

                    // Shooting PRE_ACTION
                    if (endsAtShoot(finished)) {
                        preActionIsForShooting = true;
                        nextPathIndex = finished + 1;
                        preActionTimerStarted = false;
                        preActionEntered = false;
                        state = AutoState.PRE_ACTION;
                    }

                    // Intake-only PRE_ACTION (3, 6, 9)
                    else if (finished == 3 || finished == 6 || finished == 9) {
                        preActionIsForShooting = false;
                        nextPathIndex = finished + 1;
                        preActionTimerStarted = false;
                        preActionEntered = false;
                        state = AutoState.PRE_ACTION;
                    }

                    // Normal path transition
                    else {
                        int next = finished + 1;
                        if (next > 11) {
                            state = AutoState.FINISHED;
                        } else {
                            startPath(next);
                        }
                    }
                }
                break;

            case PRE_ACTION:
                // First tick after entering PRE_ACTION
                if (!preActionEntered) {
                    preActionEntered = true;
                    poseWaitTimer.resetTimer();
                    preActionTimer.resetTimer();
                    preActionTimerStarted = false;
                }

                if (preActionIsForShooting) {
                    // SHOOTING PRE_ACTION
                    if (!preActionTimerStarted) {
                        double dist = distanceToShootPose();
                        double headingErr = headingErrorRad(follower.getPose().getHeading(), SHOOT_HEADING_RAD);

                        boolean poseGood = dist <= START_POSE_TOLERANCE_IN && Math.abs(headingErr) <= HEADING_TOLERANCE_RAD;
                        boolean poseTimeout = poseWaitTimer.getElapsedTimeSeconds() >= PRE_ACTION_MAX_POSE_WAIT_SECONDS;

                        if (poseGood || poseTimeout) {
                            try {
                                // Snap odometry to known shoot pose (position + heading) to remove drift
                                follower.setPose(new Pose(SHOOT_POSE_X, SHOOT_POSE_Y, SHOOT_HEADING_RAD));
                            } catch (Exception ignored) {
                                // If follower lacks setPose(), remove this call
                            }
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
                    }
                    else {
                        if (preActionTimer.getElapsedTimeSeconds() >= PRE_ACTION_WAIT_SECONDS) {
                            // Start the GateController shooting sequence
                            gateController.startIntakeSequence(nowMs);

                            // Move to CLAW_ACTION where GateController + ClawController take over
                            state = AutoState.CLAW_ACTION;
                        }
                    }
                }
                else {
                    // INTAKE-ONLY PRE_ACTION (paths 3, 6, 9)
                    startIntake();
                    // Use startPath which will schedule the path if the follower is still busy.
                    startPath(nextPathIndex);
                    nextPathIndex = -1;
                    // If startPath succeeded it set state to RUNNING_PATH; otherwise it scheduled the path and we stay in PRE_ACTION
                    if (state != AutoState.RUNNING_PATH) {
                        panelsTelemetry.debug("PRE_ACTION", "Intake-only PRE_ACTION scheduled next path: " + pendingStartIndex);
                    }
                }
                break;



            case CLAW_ACTION:
                // Let GateController run its timed sequence
                boolean trigger = gateController.update(nowMs);
                if (trigger) {
                    clawController.trigger(nowMs);
                }
                clawController.update(nowMs);

                // When the gate sequence finishes, move on
                if (!gateController.isBusy()) {
                    if (nextPathIndex > 0 && nextPathIndex <= 11) {
                        // Use startPath which will schedule the path if follower is busy.
                        startPath(nextPathIndex);
                        panelsTelemetry.debug("CLAW_ACTION", "Requested start of next path " + nextPathIndex);
                        nextPathIndex = -1;
                    } else {
                        state = AutoState.FINISHED;
                    }
                }
                break;



            case FINISHED:
                // idle; do nothing. Hardware remains as last set.
                break;

            case IDLE:
            default:
                // Shouldn't be here during active OpMode; remain idle
                break;
        }
    }

    // Gate open/close by distance to shoot pose
    private void updateGate() {
        if (gateServo == null) return;
        double dist = distanceToShootPose();
        try {
            if (dist <= GATE_OPEN_TOLERANCE_IN && gateClosed) {
                gateServo.setPosition(GATE_OPEN);
                gateClosed = false;
                panelsTelemetry.debug("Gate", "Opened (dist=" + String.format("%.2f", dist) + ")");
            } else if (dist >= GATE_CLOSE_TOLERANCE_IN && !gateClosed) {
                gateServo.setPosition(GATE_CLOSED);
                gateClosed = true;
                panelsTelemetry.debug("Gate", "Closed (dist=" + String.format("%.2f", dist) + ")");
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Gate", "updateGate error: " + e.getMessage());
        }
    }

    private double headingErrorRad(double current, double target) {
        double err = target - current;
        while (err > Math.PI) err -= 2 * Math.PI;
        while (err < -Math.PI) err += 2 * Math.PI;
        return err;
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 122.000), new Pose(48.000, 96.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(130))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(44.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 84.000), new Pose(24.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 84.000), new Pose(48.000, 96.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130)) // *** CHANGED: completed line
                    .build();


            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(46.000, 58.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(46.000, 58.000), new Pose(15.000, 58.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 58.000), new Pose(48.000, 96.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(45.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.000, 36.000), new Pose(14.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.000, 36.000), new Pose(48.000, 96.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(20.000, 122.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(135))
                    .build();
        }
    }
    private Pose getManualPathEnd(int idx) {
        switch (idx) {
            case 1: return new Pose(48, 96, 0);
            case 2: return new Pose(44, 84, 0);
            case 3: return new Pose(24, 84, 0);
            case 4: return new Pose(48, 96, 0);
            case 5: return new Pose(46, 58, 0);
            case 6: return new Pose(15, 58, 0);
            case 7: return new Pose(48, 96, 0);
            case 8: return new Pose(45, 36, 0);
            case 9: return new Pose(14, 36, 0);
            case 10: return new Pose(48, 96, 0);
            case 11: return new Pose(20, 122, 0);
            default: return null;
        }
    }

}