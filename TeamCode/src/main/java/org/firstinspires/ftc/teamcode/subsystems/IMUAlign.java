package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUAlign {
    private boolean isBlue;
    public Follower follower;
    private double imuAlignAngle;
    private  double lastImuTime;
    private double targetRPM;
    private double currentTime;
    private double lastImuError = 0;
    final double kP = 0.02;      // Proportional gain
    final double kD = 0.0025;     // Derivative gain for damping
    final double minPower = 0.07;  // Minimum power to overcome friction
    final double angleTolerance = 1.5; // Degrees, when to stop
    final double velocityTolerance = 0.2; // Not used as encoder velocity not available on DcMotor (non-Ex)
    final double alignTimeout = 1.5; // seconds
    private static final double ConstantLock = 20; //contant multiplyer for RPM conversion
    private final DcMotor frontLeftDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backRightDrive;
    private IMU imu;
    private  double turnPower;
    public IMUAlign (
            boolean isBlue,
            double imuAlignAngle,
            double lastImuTime,
            double targetRPM,
            DcMotor frontLeftDrive,
            DcMotor backLeftDrive,
            DcMotor frontRightDrive,
            DcMotor backRightDrive,
            IMU imu
    ) {
        this.isBlue = isBlue;
        this.imuAlignAngle = imuAlignAngle;
        this.lastImuTime = lastImuTime;
        this.targetRPM = targetRPM;
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.backLeftDrive = backLeftDrive;
        this.backRightDrive = backRightDrive;
        this.imu = imu;
    };
    public void IMUOn(double currentTime) {
        // --- Improved align logic with P+D controller and braking ---
        Pose p = follower.getPose();
        double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (isBlue) {
            imuAlignAngle = -Math.toDegrees(Math.atan2(138 - p.getY(), 8 - p.getX()));
        } else {
            imuAlignAngle = -Math.toDegrees(Math.atan2(138 - p.getY(), 136 - p.getX()));
        }
        imuAlignAngle = imuAlignAngle + 90;
        double error = imuAlignAngle - imuAngle;
        // Derivative calculation for braking (damping)
        double deltaTime = currentTime - lastImuTime;
        double derivative = 0;
        if (deltaTime > 0) derivative = (error - lastImuError) / deltaTime;

        turnPower = kP * error + kD * derivative;

        // Clamp for minimum power for large error, but stop for small error
        if (Math.abs(turnPower) < minPower && Math.abs(error) > angleTolerance) {
            turnPower = Math.signum(turnPower) * minPower;
        }
        if (Math.abs(error) < angleTolerance) {
            turnPower = 0;
        }

        // Apply rotation power

        // Stop aligning if tolerance reached or timeout
                /*if (Math.abs(error) < angleTolerance || alignTimer.seconds() > alignTimeout) {
                    // Brake robot at heading
                    double StartTime=currentTime;
                    if (currentTime>=StartTime+150) {
                        frontLeftDrive.setPower(0);
                        backLeftDrive.setPower(0);
                        frontRightDrive.setPower(0);
                        backRightDrive.setPower(0);
                    }
                    else {
                        xImuAlignActive = false;
                    }
                }*/
        lastImuError = error;
        lastImuTime = currentTime;
        //continue; // skip rest of loop when aligning
    }
    public double getTurnPower(){
        return turnPower;
    }
    public double IMUTarget(double X, double Y) {
        return ConstantLock * Math.sqrt((X * X) + ((144-Y) * (144-Y)));
    }
    public void IMUOff(){
        turnPower = 0;
    }
}
