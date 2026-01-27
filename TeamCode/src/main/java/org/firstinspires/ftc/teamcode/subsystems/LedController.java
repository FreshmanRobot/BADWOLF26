package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class LedController {
    private final LED backLedR;
    private final LED backLedG;
    private final LED sideLedR;
    private final LED sideLedG;

    public LedController (LED backLedR, LED backLedG, LED sideLedR, LED sideLedG) {
        this.backLedR = backLedR;
        this.backLedG = backLedG;
        this.sideLedR = sideLedR;
        this.sideLedG = sideLedG;
    }

    public void ledState(boolean closed) {
        if (!closed) {
            backLedG.on();
            sideLedG.on();
            backLedR.off();
            sideLedR.off();
        } else {
            backLedR.on();
            sideLedR.on();
            backLedG.off();
            sideLedG.off();
        }
    }
}