package org.firstinspires.ftc.teamcode.autopaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.OdometryPosition;

@Autonomous(name = "EthanOdoTest", group = "Autonomous")
public class EthanOdoTest extends OpMode{
    DcMotor frontLeftDrive,backLeftDrive,frontRightDrive,backRightDrive,intakeMotor,shooter,shooter2;
    OdometryPosition odometryPosition;

    @Override
    public void init() {
        //wheels
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void init_loop() {
        //uhhh
    }
    @Override
    public void start() {
        odometryPosition = new OdometryPosition(frontLeftDrive, backLeftDrive, 0, 0, 0);
    }
    @Override
    public void loop() {
        //calc is slang for calculator
        odometryPosition.OdoCalc();
        telemetry.addData("XPos: ", odometryPosition.X);
        telemetry.addData("YPos: ", odometryPosition.Y);
        telemetry.addData("Angle: ", odometryPosition.W);
        telemetry.addData("Dist: ", odometryPosition.D);

    }
}
