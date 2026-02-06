package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.LED;

public class Blink182 {
    private final LED backLedR;
    private final LED sideLedR;

    public Blink182 (LED backLedR, LED sideLedR) {
        this.backLedR = backLedR;
        this.sideLedR = sideLedR;
    }

    public void reverseLed(boolean closed) {
        if (!closed) {
            backLedR.off();
            sideLedR.off();
        } else {
            backLedR.on();
            sideLedR.on();
        }
    }
}
