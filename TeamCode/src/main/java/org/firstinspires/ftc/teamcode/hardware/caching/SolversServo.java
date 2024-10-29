package org.firstinspires.ftc.teamcode.hardware.caching;

import static org.firstinspires.ftc.teamcode.hardware.System.round;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * A wrapper servo class that provides caching to avoid unnecessary setPosition() calls.
 * Credit to team FTC 22105 (Runtime Terror) for the base class, we just modified it
 */

public class SolversServo {
    // Set to 2 at the start so that any pos will update it
    private double lastPos = 2;
    private final Servo servo;

    private double posThreshold = 0.0;

    public SolversServo(Servo servo, double posThreshold) {
        this.servo = servo;
        this.posThreshold = posThreshold;
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    public void setPosition(double pos) {
        if (Math.abs(this.lastPos - pos) > this.posThreshold) {
            lastPos = pos;
            servo.setPosition(pos);
        }
    }

    public double getPosition() {
        return round(lastPos, 2);
    }

    public double getPosition(int places) {
        return round(lastPos, places);
    }
}