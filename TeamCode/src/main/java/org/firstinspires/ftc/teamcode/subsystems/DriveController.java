package org.firstinspires.ftc.teamcode.subsystems;

/*
  DriveController.java
  --------------------
  Minimal wrapper that performs the exact same mecanum mixing and normalization
  that was originally in secondexperimentalHORS. NO smoothing, NO changes to logic —
  only an extraction to keep the OpMode uncluttered.

  Usage:
    DriveController drive = new DriveController(frontLeft, frontRight, backLeft, backRight);
    drive.setDrive(axial, lateral, yaw, speedScale);
*/

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveController {

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    public DriveController(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    /**
     * Compute wheel powers using the same mixing & normalization as your original OpMode.
     *
     * @param axial forward/backward input (positive forward)
     * @param lateral left/right input (positive right)
     * @param yaw rotation input (positive clockwise)
     * @param speedScale overall scaling 0..1 (1 = full speed)
     */
    public void setDrive(double axial, double lateral, double yaw, double speedScale) {
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize so no value exceeds 1.0 (exactly as your original code)
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Apply speed scaling (same as original)
        frontLeftPower  *= speedScale;
        frontRightPower *= speedScale;
        backLeftPower   *= speedScale;
        backRightPower  *= speedScale;

        // Write to motors
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /** Stop the drive immediately. */
    public void stop() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }
}