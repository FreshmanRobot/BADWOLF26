package org.firstinspires.ftc.teamcode.autopaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * PointLinearAuto
 *
 * LinearOpMode that builds all paths inline and follows them sequentially.
 * - Consolidates path-building and execution into a single autonomous class.
 * - No shooter/intake/claw logic — robot simply follows the 11 Bezier paths.
 * - Per-path timeout prevents getting stuck on any single path.
 *
 * Requirements:
 * - Constants.createFollower(hardwareMap) must exist and return a configured Pedropathing Follower.
 */
@Autonomous(name = "Point Linear Auto", group = "Autonomous")
public class PointLinearAuto extends LinearOpMode {

    private Follower follower;
    private PathChain[] paths;

    // Timeout per path (ms) to avoid getting stuck indefinitely
    private static final long PATH_TIMEOUT_MS = 20_000L;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing follower and building paths...");
        telemetry.update();

        // Create follower (expects your project to provide this helper)
        try {
            follower = Constants.createFollower(hardwareMap);
        } catch (Exception e) {
            follower = null;
            telemetry.addData("Error", "Failed to create follower: " + e.getMessage());
            telemetry.update();
        }

        if (follower == null) {
            telemetry.addData("Init", "Follower not available. Abort.");
            telemetry.update();
            // Wait for user to see the message; still block until start so user can abort if needed.
            waitForStart();
            return;
        }

        // Build the 11 Bezier paths (same points/headings as original pedroauto)
        paths = new PathChain[11];

        paths[0] = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(20.000, 122.000), new Pose(48.000, 96.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        paths[1] = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(44.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        paths[2] = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(44.000, 84.000), new Pose(24.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        paths[3] = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(24.000, 84.000), new Pose(48.000, 96.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        paths[4] = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(46.000, 57.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        paths[5] = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(46.000, 57.000), new Pose(15.000, 57.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        paths[6] = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(15.000, 57.000), new Pose(48.000, 96.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        paths[7] = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(43.000, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        paths[8] = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(43.000, 36.000), new Pose(16.000, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        paths[9] = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(16.000, 36.000), new Pose(48.000, 96.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        paths[10] = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(20.000, 122.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        telemetry.addData("Init", "Paths built");
        telemetry.update();

        // Wait for the play button
        waitForStart();

        if (!opModeIsActive()) return;

        // Set starting pose (same as original)
        follower.setStartingPose(new Pose(20.0, 122.0, Math.toRadians(135.0)));

        // Sequentially follow each path
        for (int i = 0; i < paths.length && opModeIsActive(); i++) {
            telemetry.addData("Path", "Starting path " + (i + 1));
            telemetry.update();

            try {
                follower.followPath(paths[i]);
            } catch (Exception e) {
                telemetry.addData("Error", "followPath failed for path " + (i + 1) + ": " + e.getMessage());
                telemetry.update();
                break;
            }

            // Wait for follower to finish or timeout or opMode stop
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && follower.isBusy() && timer.milliseconds() < PATH_TIMEOUT_MS) {
                follower.update();
                // Yield a bit
                sleep(10);
            }

            if (!opModeIsActive()) break;

            if (follower.isBusy()) {
                telemetry.addData("Timeout", "Path " + (i + 1) + " timed out after " + PATH_TIMEOUT_MS + "ms; advancing.");
                telemetry.update();
                // If your follower supports canceling/clearing a path, call it here.
            } else {
                telemetry.addData("Path", "Completed path " + (i + 1));
                telemetry.update();
            }
        }

        telemetry.addData("Status", "Sequence complete or stopped");
        telemetry.update();

        // keep final telemetry visible briefly
        sleep(250);
    }
}