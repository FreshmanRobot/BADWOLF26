package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeAndShooting {
    private final DcMotor in;
    private final DcMotor shoot1;
    private final DcMotor shoot2;
    private final Servo claw;
    private final Servo gate;

    //all this for the bumahh shooting thing
    private double Prev = 0;
    private double Curr = 0;
    private double Speed = 0;
    private boolean ballthroughcurr = false;
    private boolean ballthroughprev = false;
    private int ballcount = 0;
    private int clawtimer = 0;
    public boolean finished = false;

    public IntakeAndShooting (DcMotor in, DcMotor shoot1, DcMotor shoot2, Servo claw, Servo gate) {
        this.in = in;
        this.shoot1 = shoot1;
        this.shoot2 = shoot2;
        this.claw = claw;
        this.gate = gate;
    }

    public void run (int status) {
        Curr = shoot1.getCurrentPosition();
        Speed = Curr-Prev;
        Prev = Curr;
        switch (status) {
            case 0:
                ballcount = 0;
                power(0,0,Speed,false,true);
                break;
            case 1:
                ballcount = 0;
                power(1,0,Speed,false,true);
                break;
            case 2:
                power(1,3100,Speed,true,false);
                break;
        }
    }

    private void power (double inP, double goalspeed, double speed, boolean clawP, boolean gateP) {
        shoot1.setPower(speed);
        shoot2.setPower(speed);

        if (Math.abs(goalspeed-speed) < 100) {
            in.setPower(inP);
        } else {
            in.setPower(0);
            if (speed < goalspeed) {
                ballthroughcurr = true;
                clawtimer = 0;
            }
        }

        if (ballthroughcurr && !ballthroughprev) {
            ballcount++;
        }
        ballthroughprev = ballthroughcurr;

        if (clawP && ballcount == 2) {
            if (clawtimer > 600) {
                //claw up
                claw.setPosition(0.63);
                finished = (clawtimer > 700);
            } else {
                claw.setPosition(0.2);
            }
            clawtimer++;
        } else {
            //claw down
            claw.setPosition(0.2);
            finished = (clawP && ballcount == 3);
        }

        if (gateP) {
            //gate closed
            gate.setPosition(0.45);
        } else {
            //gate open
            gate.setPosition(0.28);
        }
    }
}