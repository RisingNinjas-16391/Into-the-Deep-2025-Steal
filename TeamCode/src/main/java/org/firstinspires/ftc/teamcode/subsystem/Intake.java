package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private double target;
    // Between retracted and extended
    public boolean extendoRetracted;
    // Between transfer and intake position
    public int wristIndex = 3;
    public enum ExtendoState {
        FULL_EXTENSION,
        HALF_EXTENSION,
        NO_EXTENSION;
        public enum IntakePivotState {
            READY_INTAKE,
            INTAKE,
            TRANSFER,
            MIDDLE_HOLD
        }
    }

    public static ExtendoState extendoState;
    public static ExtendoState.IntakePivotState intakePivotState;
    private static final PIDFController extendoPIDF = new PIDFController(0,0,0, 0);

    public void init() {
        setExtendoTarget(0);
    }

    public void autoExtendoSetPower() {
        double extendoPower = extendoPIDF.calculate(robot.liftEncoder.getPosition(), this.target);
        robot.extension.setPower(extendoPower);

        // Extendo is only retracted it has reached a target of 0
        extendoRetracted = ((this.target <= 0)) && (this.reached());

        if (extendoRetracted) {
            extendoState = ExtendoState.FULL_EXTENSION;
        }
    }

    // Returns if extendo has reached the target
    public boolean reached() {
        return (extendoPIDF.atSetPoint());
    }

    public void stopSlide() {
        robot.liftTop.setPower(0);
        robot.liftBottom.setPower(0);
    }

    public void setPivotServo(double target) {
        robot.leftIntakePivot.setPosition(target);
        robot.rightIntakePivot.setPosition(target);
    }

    public void openClaw() {
        robot.intakeClaw.setPosition(INTAKE_CLAW_OPEN_POS);
    }

    public void closeClaw() {
        robot.intakeClaw.setPosition(INTAKE_CLAW_CLOSE_POS);
    }

    public void setWrist(double target) {
        robot.wrist.setPosition(target);
    }

    public void moveWrist() {
        robot.wrist.setPosition(WRIST_POSITIONS[Math.max(Math.min(wristIndex, 5), 0)]);
    }

    public void setWristTransfer() {
        if (intakePivotState.equals(ExtendoState.IntakePivotState.MIDDLE_HOLD)) {
            robot.wrist.setPosition(WRIST_TRANSFER_POS);
        }
    }
    public void setWristIntake() {
        if (!intakePivotState.equals(ExtendoState.IntakePivotState.MIDDLE_HOLD)) {
            robot.wrist.setPosition(WRIST_INTAKE_POS);
        }
        intakePivotState = ExtendoState.IntakePivotState.INTAKE;
    }
    public void setExtendoTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_EXTENDO_EXTENSION), 0);
        extendoPIDF.setSetPoint(this.target);
    }

    public SampleDetected sampleDetected() {
        if (intakePivotState.equals(ExtendoState.IntakePivotState.INTAKE)) {
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
