package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private double target;
    // Between retracted and extended
    public boolean extendoRetracted;
    // Between transfer and intake position
    public boolean trayTransfer;
    public enum IntakeState {
        INTAKE,
        TRANSFER
    }

    public static IntakeState intakeState;
    private static final PIDFController extendoPIDF = new PIDFController(0,0,0, 0);

    public void init() {
        setExtendoTarget(0);
    }

    public void autoExtendoSetPower() {
        double extendoPower = extendoPIDF.calculate(robot.liftEncoder.getPosition(), this.target);
        robot.extension.setPower(extendoPower);

        // Extendo is only retracted it has reached a target of 0
        extendoRetracted = ((this.target <= 0)) && (this.reached());
    }

    // Returns if extendo has reached the target
    public boolean reached() {
        return (extendoPIDF.atSetPoint());
    }

    public void setPivotServo(double target) {
        robot.leftPivot.setPosition(target);
        robot.rightPivot.setPosition(target);
    }

    public void openClaw() {
        robot.claw.setPosition(CLAW_OPEN_POS);
    }

    public void closeClaw() {
        robot.claw.setPosition(CLAW_CLOSE_POS);
    }

    public void setWrist(double target) {
        robot.wrist.setPosition(target);
    }

    public void setExtendoTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_EXTENDO_EXTENSION), 0);
        extendoPIDF.setSetPoint(this.target);
    }

    public SampleDetected sampleDetected() {
        if (intakeState == IntakeState.INTAKE) {
            robot.colorSensor.enableLed(true);

            int red = robot.colorSensor.red();
            int green = robot.colorSensor.green();
            int blue = robot.colorSensor.blue();

            double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

            if (distance < 3.5) {
                if (red >= green && red >= blue) {
                    return SampleDetected.RED;
                } else if (green >= red && green >= blue) {
                    return SampleDetected.YELLOW;
                } else {
                    return SampleDetected.BLUE;
                }
            }
        }
        return currentSample;
    }

    @Override
    public void periodic() {
        autoExtendoSetPower();
    }
}
