package org.firstinspires.ftc.teamcode.autopaths;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.MecanumCalc;
import org.firstinspires.ftc.teamcode.subsystems.OdometryPosition;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndShooting;
import org.firstinspires.ftc.teamcode.subsystems.Blink182;

@Autonomous(name = "EthanDoesASillyLittleCoding", group = "Autonomous")
public class EthanDoesASillyLittleCoding extends OpMode{
    DcMotor frontLeftDrive,backLeftDrive,frontRightDrive,backRightDrive,intakeMotor,shooter,shooter2;
    Servo clawServo, gateServo;
    LED backLedR, sideLedR;

    MecanumCalc mecanumCalc;
    OdometryPosition odometryPosition;
    IntakeAndShooting intakeAndShooting;
    Blink182 blink182;

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
        //intake
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        //shooter
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotor.Direction.REVERSE);
        //servos
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        //leds
        backLedR = hardwareMap.get(LED.class, "backLedR");
        sideLedR = hardwareMap.get(LED.class, "sideLedR");
        //uhm
        mecanumCalc = new MecanumCalc();
        odometryPosition = new OdometryPosition(frontLeftDrive, backLeftDrive, 0, 0);
        intakeAndShooting = new IntakeAndShooting(intakeMotor,shooter,shooter2,clawServo,gateServo);
        blink182 = new Blink182(backLedR, sideLedR);
    }

    @Override
    public void init_loop() {
        //uhhh
    }
    @Override
    public void start() {
        //uhhhh
    }
    @Override
    public void loop() {
        //calc is slang for calculator
        odometryPosition.OdoCalc();
        mecanumCalc.calculate(odometryPosition.X,odometryPosition.Y,odometryPosition.W, intakeAndShooting.finished);
        intakeAndShooting.run(mecanumCalc.status);
        blink182.reverseLed(mecanumCalc.ledon);
        //set powers
        backLeftDrive.setPower(MecanumCalc.bLPower);
        backRightDrive.setPower(MecanumCalc.bRPower);
        frontLeftDrive.setPower(MecanumCalc.fLPower);
        frontRightDrive.setPower(MecanumCalc.fRPower);

    }
}
