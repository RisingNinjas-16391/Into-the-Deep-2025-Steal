package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.*;

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
        public enum WristState {
            INTAKE,
            TRANSFER,
            OTHER
        }
    }

    public static ExtendoState extendoState;
    public static ExtendoState.IntakePivotState intakePivotState = ExtendoState.IntakePivotState.TRANSFER;
    public static ExtendoState.WristState WristState = ExtendoState.WristState.TRANSFER;
    private static final PIDFController extendoPIDF = new PIDFController(0.023,0,0, 0);

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

    public void setExtendoTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_EXTENDO_EXTENSION), 0);
        extendoPIDF.setSetPoint(this.target);
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
        robot.wrist.setPosition(WRIST_TRANSFER_POS);

    }
    public void setWristIntake() {
        robot.wrist.setPosition(WRIST_INTAKE_POS);
    }

    public void openTray() {
        robot.trayServo.setPosition(TRAY_OPEN_POS);
    }

    public void closeTray() {
        robot.trayServo.setPosition(TRAY_CLOSE_POS);
    }

    public SampleDetected sampleDetected() {
        robot.colorSensor.enableLed(true);

        int red = robot.colorSensor.red();
        int green = robot.colorSensor.green();
        int blue = robot.colorSensor.blue();

        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

        if (distance < 1.5) {
            if (red >= green && red >= blue) {
                return SampleDetected.RED;
            } else if (green >= red && green >= blue) {
                return SampleDetected.YELLOW;
            } else {
                return SampleDetected.BLUE;
            }
        }
        else {
            return SampleDetected.NONE;
        }
    }

    public double colorSensorDistance() {
        robot.colorSensor.enableLed(true);

        return(robot.colorSensor.getDistance(DistanceUnit.CM));
    }

    @Override
    public void periodic() {
        autoExtendoSetPower();
    }
}
