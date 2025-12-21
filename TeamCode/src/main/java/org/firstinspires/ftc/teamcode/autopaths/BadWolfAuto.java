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
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.paths.Path;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.TurretController;

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
@Autonomous(name = "BW Blue Side 12 Ball", group = "Autonomous")
@Configurable
public class BadWolfAuto extends OpMode {

    // Shooter / feeder hardware
    private DcMotor shooter = null;
    private DcMotor intakeMotor = null;
    private CRServo transfer = null;
    private Servo clawServo = null;

    // Shooter RPM estimation (encoder ticks -> RPM) - kept for telemetry only
    private static final double TICKS_PER_REV = 537.6;
    private double currentRPM = 0.0;
    private int lastShooterPosition = 0;
    private long lastShooterTimeMs = 0L;

    // Controller tuning (not used to repeatedly set power anymore)
    private double targetRPM = 120.0;    // informational
    private double emaAlpha = 0.20;      // smoothing for telemetry RPM only

    // Timings / distances (ms) - increased for farther backing and longer feed
    private static final long DRIVE_BACK_MS = 2000;       // increased to back up farther
    private static final double DRIVE_POWER = 0.35;
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
    private static final double SHOOTER_APPLIED_POWER = 0.55; // tune this to reach ~120 RPM on your robot


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Starting pose of the robot
    private final Pose startPose   = new Pose(20.000, 122.000, Math.toRadians(135));

    // Hub / center point you keep returning to (48, 96)
    private final Pose scorePose     = new Pose(48.000, 96.000, Math.toRadians(130));

    // First cycle: go-to and pickup
    private final Pose goTo1Pose   = new Pose(44.000, 84.000, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(24.000, 84.000, Math.toRadians(180));

    // Second cycle: go-to and pickup
    private final Pose goTo2Pose   = new Pose(46.000, 57.000, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(15.000, 57.000, Math.toRadians(180));

    // Third cycle: go-to and pickup
    private final Pose goTo3Pose   = new Pose(43.000, 36.000, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(16.000, 36.000, Math.toRadians(180));

    // Final return pose (back to start)
    private final Pose endPose     = new Pose(20.000, 122.000, Math.toRadians(135));

    private Path scorePreload;
    private PathChain grabPickup1, goTo1, scorePickup1, grabPickup2, goTo2, scorePickup2, grabPickup3, goTo3, scorePickup3, goToEnd;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        goTo1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, goTo1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goTo1Pose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(goTo1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(goTo1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        goTo2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, goTo2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goTo2Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(goTo2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(goTo2Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        goTo3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, goTo3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goTo3Pose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(goTo3Pose, pickup3Pose))
                .setLinearHeadingInterpolation(goTo3Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        goToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // -------------------------
            // PRELOAD
            // -------------------------
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // After scoring preload, go to first goTo1 position
                    follower.followPath(goTo1, true);
                    setPathState(2);
                }
                break;

            // -------------------------
            // CYCLE 1
            // -------------------------
            case 2:
                if (!follower.isBusy()) {
                    // Move from goTo1Pose → pickup1Pose
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    // Move from pickup1Pose → scorePose
                    follower.followPath(scorePickup1, true);
                    setPathState(4);
                }
                break;

            // -------------------------
            // CYCLE 2
            // -------------------------
            case 4:
                if (!follower.isBusy()) {
                    // Move from scorePose → goTo2Pose
                    follower.followPath(goTo2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    // Move from goTo2Pose → pickup2Pose
                    follower.followPath(grabPickup2, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    // Move from pickup2Pose → scorePose
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }
                break;

            // -------------------------
            // CYCLE 3
            // -------------------------
            case 7:
                if (!follower.isBusy()) {
                    // Move from scorePose → goTo3Pose
                    follower.followPath(goTo3, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    // Move from goTo3Pose → pickup3Pose
                    follower.followPath(grabPickup3, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    // Move from pickup3Pose → scorePose
                    follower.followPath(scorePickup3, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    // Move from pickup3Pose → scorePose
                    follower.followPath(goToEnd, true);
                    setPathState(11);
                }
                break;
            // -------------------------
            // END
            // -------------------------
            case 11:
                if (!follower.isBusy()) {
                    // Stop running new paths
                    setPathState(-1);
                }
                break;
        }
    }


    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        shooter.setPower(0);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        shooter.setPower(1);
        setPathState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }



    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}