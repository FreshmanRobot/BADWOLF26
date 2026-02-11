package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BWTEST", group = "Autonomous")
@Configurable
public class BWTEST extends OpMode {

    private final Pose startPose  = new Pose(20, 122, Math.toRadians(135));
    private final Pose scorePose  = new Pose(48, 96, Math.toRadians(130));

    private final Pose goTo1      = new Pose(44, 84, Math.toRadians(180));
    private final Pose pickup1    = new Pose(24, 84, Math.toRadians(180));

    private final Pose goTo2      = new Pose(48, 96, Math.toRadians(130));
    private final Pose pickup2    = new Pose(46, 58, Math.toRadians(180));

    private final Pose goTo3      = new Pose(15, 58, Math.toRadians(180));
    private final Pose pickup3    = new Pose(48, 96, Math.toRadians(130));

    private Path scorePreload;
    private PathChain goToPickup1, grabPickup1, scorePickup1, goToPickup2, grabPickup2, scorePickup2, goToPickup3, grabPickup3, scorePickup3;

    public void buildPaths() {

        /* Preload scoring path */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* -------------------- PICKUP 1 -------------------- */

        // score → goTo1
        goToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, goTo1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goTo1.getHeading())
                .build();

        // goTo1 → pickup1
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(goTo1, pickup1))
                .setLinearHeadingInterpolation(goTo1.getHeading(), pickup1.getHeading())
                .build();

        // pickup1 → score
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, scorePose))
                .setLinearHeadingInterpolation(pickup1.getHeading(), scorePose.getHeading())
                .build();


        /* -------------------- PICKUP 2 -------------------- */

        // score → goTo2
        goToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, goTo2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goTo2.getHeading())
                .build();

        // goTo2 → pickup2
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(goTo2, pickup2))
                .setLinearHeadingInterpolation(goTo2.getHeading(), pickup2.getHeading())
                .build();

        // pickup2 → score
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2, scorePose))
                .setLinearHeadingInterpolation(pickup2.getHeading(), scorePose.getHeading())
                .build();


        /* -------------------- PICKUP 3 -------------------- */

        // score → goTo3
        goToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, goTo3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goTo3.getHeading())
                .build();

        // goTo3 → pickup3
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(goTo3, pickup3))
                .setLinearHeadingInterpolation(goTo3.getHeading(), pickup3.getHeading())
                .build();

        // pickup3 → score
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3, scorePose))
                .setLinearHeadingInterpolation(pickup3.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                // Run preload
                follower.followPath(scorePreload);
                //shooter
                long nowMs = System.currentTimeMillis();
                gateController.startIntakeSequence(nowMs);
                setPathState(1);
                break;

            /* -------------------- PICKUP 1 -------------------- */

            case 1:
                if (!follower.isBusy()) {
                    // score → goTo1
                    follower.followPath(goToPickup1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    // goTo1 → pickup1
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    // pickup1 → score
                    follower.followPath(scorePickup1, true);
                    setPathState(4);
                }
                break;

            /* -------------------- PICKUP 2 -------------------- */

            case 4:
                if (!follower.isBusy()) {
                    // score → goTo2
                    follower.followPath(goToPickup2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    // goTo2 → pickup2
                    follower.followPath(grabPickup2, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    // pickup2 → score
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }
                break;

            /* -------------------- PICKUP 3 -------------------- */

            case 7:
                if (!follower.isBusy()) {
                    // score → goTo3
                    follower.followPath(goToPickup3, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    // goTo3 → pickup3
                    follower.followPath(grabPickup3, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    // pickup3 → score
                    follower.followPath(scorePickup3, true);
                    setPathState(10);
                }
                break;

            /* -------------------- DONE -------------------- */

            case 10:
                if (!follower.isBusy()) {
                    // Stop running new paths
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Example subsystems — rename to match your robot
    private DcMotor shooter, shooter2, intakeMotor;

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private Servo clawServo = null;
    private Servo leftHoodServo = null;
    private Servo gateServo = null;

    // Gate/Intake constants - UPDATED POSITIONS
    private static final double GATE_OPEN = 0.28;
    private static final double GATE_CLOSED = 0.45;
    private static final long INTAKE_DURATION_MS = 1200;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 100;
    private static final double INTAKE_SEQUENCE_POWER = 1.0;

    private double targetRPM= 3300;
    private boolean atTargetLast = false;

    private double leftHoodPosition = 0.9;

    // Claw constants
    private static final double CLAW_OPEN = 0.2;
    private static final double CLAW_CLOSED = 0.63;
    private static final long CLAW_CLOSE_MS = 500L;

    FlywheelController flywheel;
    ClawController clawController;
    GateController gateController;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20, 122, Math.toRadians(135)));

        // 2. RESET WHEELS (RECALIBRATE DEAD WHEELS)
        try {
            // Reset drive encoders (assuming typical 4-motor drive)
            DcMotor fL = hardwareMap.get(DcMotor.class, "leftFront");
            DcMotor fR = hardwareMap.get(DcMotor.class, "rightFront");
            DcMotor bL = hardwareMap.get(DcMotor.class, "leftBack");
            DcMotor bR = hardwareMap.get(DcMotor.class, "rightBack");

            fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            telemetry.addLine("Drive reset failed, checking names...");
        }

        //shooters
        try {
            shooter = hardwareMap.get(DcMotor.class, "shooter");
            shooter.setDirection(DcMotor.Direction.FORWARD);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setPower(0.0);

            shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
            shooter2.setDirection(DcMotor.Direction.REVERSE);
            shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter2.setPower(0.0);

        } catch (Exception e) {
        }

        // --- Intake & compression hardware (same names as teleop) ---
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            clawServo.setPosition(CLAW_CLOSED);
            leftHoodServo = hardwareMap.get(Servo.class, "hoodServo");
            leftHoodServo.setPosition(leftHoodPosition);
            gateServo = hardwareMap.get(Servo.class, "gateServo");

            // Direction per request
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);

            // Set defaults (same as teleop off state)
            intakeMotor.setPower(0.0);

        } catch (Exception e) {
        }


        clawController = new ClawController(clawServo, CLAW_OPEN, CLAW_CLOSED, CLAW_CLOSE_MS);
        gateController = new GateController(
                gateServo, intakeMotor,
                GATE_OPEN, GATE_CLOSED,
                INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                INTAKE_SEQUENCE_POWER
        );

        // Initialize at the Open position
        gateController.setGateClosed(false);
        gateServo.setPosition(GATE_OPEN);

        // Try to obtain a voltage sensor if available (optional)
        VoltageSensor voltageSensor = null;
        try {
            if (hardwareMap.voltageSensor.iterator().hasNext()) {
                voltageSensor = hardwareMap.voltageSensor.iterator().next();
            }
        } catch (Exception ignored) {
        }


        follower.setPose(new Pose(20, 122, Math.toRadians(135)));

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        buildPaths();
        follower.setStartingPose(startPose);


        flywheel = new FlywheelController(shooter, shooter2, telemetry, voltageSensor);
        flywheel.setTargetRpm(targetRPM);
        flywheel.setShooterOn(true);

        telemetry.addLine("BWTEST Initialized");
        telemetry.update();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        flywheel.setTargetRpm(targetRPM);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);
        flywheel.toggleShooterOn();
        setPathState(0);
    }

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

    // Simple sleep helper
    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }



}