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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
 * BWBlueAuto – stabilized heading/pose lock at shoot pose + gate servo fix.
 */
@Autonomous(name = "Pedro Blue Far", group = "Autonomous")
@Configurable
public class BlueFarPedro extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // heading convergence for non‑turret robot
    private static final double SHOOT_HEADING_RAD_1 = Math.toRadians(110);
    private static final double SHOOT_HEADING_RAD_3 = Math.toRadians(110);
    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(4); // ~4°

    private enum AutoState { IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, PRE_ACTION, INTAKE_WAIT, CLAW_ACTION, FINISHED }
    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    private Timer intakeTimer;
    private static final double INTAKE_WAIT_SECONDS = 2.2;

    private Timer timedIntakeTimer;
    private static final double TIMED_INTAKE_SECONDS = 0.5;
    private boolean timedIntakeActive = false;

    private double targetRPM = 4000;

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

    private DcMotor shooterMotor, shooter2;
    private boolean shooterOn = false;

    private BNO055IMU hubImu = null;
    private BNO055IMU imu = null;

    private FlywheelController flywheel;
    private GateController gateController;
    private ClawController clawController;

    private long lastShooterPosition = 0;
    private long lastShooterTime = 0;

    private DcMotor intakeMotor;
    private CRServo transferMotor; // unused but kept
    private Servo gateServo = null;
    private Servo leftHoodServo = null;
    private static final double LEFT_HOOD_POSITION = 0.43;

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive; // unused but kept

    private Servo clawServo;

    private static final double INTAKE_ON_POWER = 1.0;
    private static final double GATE_OPEN = 0.28;
    private static final double GATE_CLOSED = 0.60;

    private static final long INTAKE_DURATION_MS = 800;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 100;
    private static final double INTAKE_SEQUENCE_POWER = 1.0;
    private int intakeSegmentEnd = -1;

    private static final double SHOOT_POSE_X_1 = 60.0;
    private static final double SHOOT_POSE_Y_1 = 10.0;
    private static final double SHOOT_POSE_X_3 = 60.0;
    private static final double SHOOT_POSE_Y_3 = 10.0;
    private static final double START_POSE_TOLERANCE_IN = 6.0;


    public BlueFarPedro() {}

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        intakeTimer = new Timer();
        timedIntakeTimer = new Timer();
        preActionTimer = new Timer();
        poseWaitTimer = new Timer();
        nextPathIndex = -1;
        intakeSegmentEnd = -1;
        preActionTimerStarted = false;
        preActionEntered = false;
        timedIntakeActive = false;

        // Map gate servo FIRST to avoid null usage below
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

        try{

            try {
                hubImu = hardwareMap.get(BNO055IMU.class, "imu");
                panelsTelemetry.debug("Init", "Found expansion hub IMU as 'imu'");
            } catch (Exception e) {
                hubImu = null;
                panelsTelemetry.debug("Init", "Expansion hub IMU 'imu' not found: " + e.getMessage());
            }

            if (imu != null) {
                BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
                imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                imu.initialize(imuParams);}else {
                panelsTelemetry.debug("Init", "No IMU found (neither 'pinpoint' nor 'imu'). Turret will not have heading data.");
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "IMU not found or failed to init: " + e.getMessage());
        }


        // Voltage sensor (optional)
        VoltageSensor voltageSensor = null;
        try {
            if (hardwareMap.voltageSensor.iterator().hasNext()) {
                voltageSensor = hardwareMap.voltageSensor.iterator().next();
            }
        } catch (Exception ignored) {}

        // Controllers (guard gate/claw creation on non-null)
        flywheel = new FlywheelController(shooterMotor, shooter2, telemetry, voltageSensor);
        flywheel.setTargetRpm(targetRPM);
        flywheel.setShooterOn(true);

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

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // Keep odometry aligned to the known start pose
        follower.setStartingPose(new Pose(60, 8, Math.toRadians(90)));
    }

    @Override
    public void start() {
        flywheel.setTargetRpm(targetRPM);
        startIntake();
        //stopIntake();
        follower.setStartingPose(new Pose(60, 8, Math.toRadians(90)));

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
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("DistToShootPose", String.format("%.2f", distanceToShootPose()));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setPower(0.0);
        if (shooter2 != null) shooter2.setPower(0.0);
        //stopIntake();
        if (clawServo != null) clawServo.setPosition(CLAW_OPEN);
        if (gateServo != null) gateServo.setPosition(GATE_CLOSED);
        state = AutoState.FINISHED;
    }

    // Intake helpers
    private void startIntake() {
        try {
            if (intakeMotor != null) intakeMotor.setPower(INTAKE_ON_POWER);
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

    //if pathIndex is listed then run PRE_ACTION
    private boolean endsAtShoot(int pathIndex) {
        return pathIndex == 1 || pathIndex == 3;
    }

    private double distanceToShootPose() {
        if (currentPathIndex==1) {
            try {
                Pose p = follower.getPose();
                double dx = p.getX() - SHOOT_POSE_X_1;
                double dy = p.getY() - SHOOT_POSE_Y_1;
                return Math.hypot(dx, dy);
            } catch (Exception e) {
                return Double.POSITIVE_INFINITY;
            }
        }
        else if (currentPathIndex==3) {
            try {
                Pose p = follower.getPose();
                double dx = p.getX() - SHOOT_POSE_X_3;
                double dy = p.getY() - SHOOT_POSE_Y_3;
                return Math.hypot(dx, dy);
            } catch (Exception e) {
                return Double.POSITIVE_INFINITY;
            }
        }
        else {
            return Double.POSITIVE_INFINITY;
        }
    }


    private void startPath(int idx) {
        if (idx < 1 || idx > 4) {
            currentPathIndex = 0;
            state = AutoState.FINISHED;
            return;
        }


        // for spikes +target
        if (idx == 2) { intakeSegmentEnd = 3; startIntake(); clawServo.setPosition(CLAW_OPEN); targetRPM=4000;}


        //shoot
        if (idx==1 || idx == 3) {
            startIntake();
            timedIntakeTimer.resetTimer();
            timedIntakeActive = true;
            panelsTelemetry.debug("TimedIntake", "Started timed intake for path " + idx);
        }

        switch (idx) {
            case 1: follower.followPath(paths.Path1); break;
            case 2: follower.followPath(paths.Path2); break;
            case 3: follower.followPath(paths.Path3); break;
            case 4: follower.followPath(paths.Path4); break;
            default: break;
        }

        currentPathIndex = idx;
        state = AutoState.RUNNING_PATH;
    }

    private void runStateMachine(long nowMs) {
        if (timedIntakeActive) {
            if (timedIntakeTimer.getElapsedTimeSeconds() >= TIMED_INTAKE_SECONDS) {
                //stopIntake();
                timedIntakeActive = false;
                intakeSegmentEnd = -1;
                panelsTelemetry.debug("TimedIntake", "Timed intake ended after " + TIMED_INTAKE_SECONDS + "s");
            } else {
                panelsTelemetry.debug("TimedIntake",
                        String.format("remaining=%.2fs", TIMED_INTAKE_SECONDS - timedIntakeTimer.getElapsedTimeSeconds()));
            }
        }

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

                    if (intakeSegmentEnd == finished) {
                        //stopIntake();
                        intakeSegmentEnd = -1;
                    }

                    if (endsAtShoot(finished)) {
                        nextPathIndex = finished + 1;
                        preActionTimerStarted = false;
                        preActionEntered = false;
                        state = AutoState.PRE_ACTION;
                    } else {
                        int next = finished + 1;
                        if (next > 4) {
                            state = AutoState.FINISHED;
                        } else {
                            startPath(next);
                        }
                    }
                }
                break;
            }

            case PRE_ACTION: {
                //go to shoot pose
                if (!preActionEntered) {
                    poseWaitTimer.resetTimer();
                    preActionTimerStarted = false;
                    preActionEntered = true;
                    panelsTelemetry.debug("PRE_ACTION", "Entered PRE_ACTION, starting pose/heading wait");
                }

                if (!preActionTimerStarted) {
                    double dist = distanceToShootPose();
                    double targetHeading;

                    if (currentPathIndex == 2)      targetHeading = SHOOT_HEADING_RAD_1;
                    else if (currentPathIndex == 3) targetHeading = SHOOT_HEADING_RAD_3;
                    else targetHeading = SHOOT_HEADING_RAD_1; // fallback

                    double headingErr = headingErrorRad(
                            follower.getPose().getHeading(),
                            targetHeading
                    );

                    boolean poseGood = dist <= START_POSE_TOLERANCE_IN && Math.abs(headingErr) <= HEADING_TOLERANCE_RAD;
                    boolean poseTimeout = poseWaitTimer.getElapsedTimeSeconds() >= PRE_ACTION_MAX_POSE_WAIT_SECONDS;

                    if (poseGood || poseTimeout) {
                        if (currentPathIndex==1) {
                            try {
                                // Snap odometry to known shoot pose (position + heading) to remove drift
                                follower.setPose(new Pose(SHOOT_POSE_X_1, SHOOT_POSE_Y_1, SHOOT_HEADING_RAD_1));
                            } catch (Exception ignored) {
                                // If follower lacks setPose(), remove this call
                            }
                        }
                        else if (currentPathIndex==3){
                            try {
                                // Snap odometry to known shoot pose (position + heading) to remove drift
                                follower.setPose(new Pose(SHOOT_POSE_X_3, SHOOT_POSE_Y_3, SHOOT_HEADING_RAD_3));
                            } catch (Exception ignored) {
                                // If follower lacks setPose(), remove this call
                            }
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
                } else {
                    if (preActionTimer.getElapsedTimeSeconds() >= PRE_ACTION_WAIT_SECONDS) {
                        //startIntake();
                        intakeMotor.setPower(0.0);
                        gateServo.setPosition(GATE_OPEN);
                        intakeTimer.resetTimer();
                        state = AutoState.INTAKE_WAIT;
                    }
                }
                break;
            }

            case INTAKE_WAIT: {
                if (flywheel.getCurrentRPM()>=3950) {
                    intakeMotor.setPower(1.0);
                }
                else {
                    intakeMotor.setPower(0.0);
                    long t = System.currentTimeMillis();
                    long i = System.currentTimeMillis();
                    while (i - t >= 200) {
                        i = System.currentTimeMillis();
                    }

                }
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_SECONDS) {
                    if (intakeSegmentEnd == -1) {
                        //stopIntake();
                    }
                    state = AutoState.CLAW_ACTION;
                }
                /*for (int x = 0; x < 3; x++) {

                    while (nowMs )
                }
                state = AutoState.CLAW_ACTION;*/
                break;
            }

            case CLAW_ACTION: {

                if (clawActionStartMs == 0) {
                    clawActionStartMs = nowMs;
                    clawServo.setPosition(CLAW_CLOSED);
                }

                if (nowMs - clawActionStartMs >= 207) {
                    //for max

                    clawServo.setPosition(CLAW_OPEN);
                    clawActionStartMs = 0;

                    if (nextPathIndex > 0 && nextPathIndex <= 4) {
                        if (gateServo != null) {
                            gateServo.setPosition(GATE_CLOSED);
                        }
                        startPath(nextPathIndex);
                        nextPathIndex = -1;
                    } else {
                        state = AutoState.FINISHED;
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
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(60.000, 8.000),
                                    new Pose(SHOOT_POSE_X_1, SHOOT_POSE_Y_1)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), SHOOT_HEADING_RAD_1)
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(SHOOT_POSE_X_1, SHOOT_POSE_Y_1),
                                    new Pose(20.000, 8.600)
                            )
                    )
                    .setLinearHeadingInterpolation(SHOOT_HEADING_RAD_1, Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(20.000, 8.600),
                                    new Pose(SHOOT_POSE_X_3, SHOOT_POSE_Y_3)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), SHOOT_HEADING_RAD_3)
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(SHOOT_POSE_X_3, SHOOT_POSE_Y_3),
                                    new Pose(30.000, 10.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}