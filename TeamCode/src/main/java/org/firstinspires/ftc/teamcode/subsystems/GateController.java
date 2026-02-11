// Java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class GateController {

    private final Servo gateServo;
    private final DcMotor intakeMotor;

    private final double gateOpenPos;
    private final double gateClosedPos;
    private final long intakeDurationMs;
    private final long clawTriggerBeforeEndMs;
    private final double intakePower;

    public boolean gateClosed = false;

    // Intake sequence state
    private boolean busy = false;
    private long sequenceStartMs = 0L;
    private boolean clawTriggeredThisCycle = false;

    public GateController(Servo gateServo,
                          DcMotor intakeMotor,
                          double gateOpenPos,
                          double gateClosedPos,
                          long intakeDurationMs,
                          long clawTriggerBeforeEndMs,
                          double intakePower) {
        this.gateServo = gateServo;
        this.intakeMotor = intakeMotor;
        this.gateOpenPos = gateOpenPos;
        this.gateClosedPos = gateClosedPos;
        this.intakeDurationMs = intakeDurationMs;
        this.clawTriggerBeforeEndMs = clawTriggerBeforeEndMs;
        this.intakePower = intakePower;
    }

    public void setGateClosed(boolean closed) {
        gateClosed = closed;
        gateServo.setPosition(closed ? gateClosedPos : gateOpenPos);
    }

    public void toggleGate() {
        setGateClosed(!gateClosed);
    }

    public boolean isBusy() {
        return busy;
    }

    public void startIntakeSequence(long nowMs) {
        busy = true;
        sequenceStartMs = nowMs;
        clawTriggeredThisCycle = false;

        // Ensure gate open for intake
        setGateClosed(false);

        // Start intake motor
        intakeMotor.setPower(-intakePower);
    }

    // Update the sequence; returns true exactly once when it is time to trigger the claw.
    public boolean update(long nowMs) {
        if (!busy) return false;

        long elapsed = nowMs - sequenceStartMs;
        long triggerAt = Math.max(0L, intakeDurationMs - clawTriggerBeforeEndMs);

        boolean shouldTriggerClaw = false;

        // Fire claw once at the desired time
        if (!clawTriggeredThisCycle && elapsed >= triggerAt) {
            clawTriggeredThisCycle = true;
            shouldTriggerClaw = true;
        }

        // End sequence
        if (elapsed >= intakeDurationMs) {
            busy = false;
            intakeMotor.setPower(0.0);
            // Optionally close gate at end
            setGateClosed(true);
        }

        return shouldTriggerClaw;
    }
}
