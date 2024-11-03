package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private double target;
    public boolean extendoReached;
    // Between retracted and extended
    public boolean extendoRetracted;
    // Between transfer and intake position
    public int wristIndex = 4;
    // Whether the claw is open or not in the current state of the claw
    public boolean clawOpen = true;

    public enum ClawState {
        INNER,
        OUTER
    }

    public ClawState clawState;

    public enum IntakePivotState {
        READY_INTAKE,
        INTAKE,
        TRANSFER,
        MIDDLE_HOLD
    }
    public enum WristState {
        INTAKE,
        TRANSFER,
        ROTATED
    }


    public static IntakePivotState intakePivotState;
    public static WristState wristState;

    private static final PIDFController extendoPIDF = new PIDFController(0.023,0,0, 0.001);

    public void init() {
        setClawState(ClawState.OUTER);
        setWrist(WristState.INTAKE);
        setPivot(IntakePivotState.MIDDLE_HOLD);
        setExtendoTarget(0);
        extendoPIDF.setTolerance(15);
    }

    public void autoUpdateExtendo() {
        double extendoPower = extendoPIDF.calculate(robot.extensionEncoder.getPosition(), this.target);

        // Just make sure it gets to fully retracted if target is 0
        if (target == 0) {
            extendoPower -= 0.1;
        }

        extendoReached = extendoPIDF.atSetPoint();
        extendoRetracted = (target <= 0) && extendoReached;

        if (extendoReached) {
            robot.extension.setPower(0);
        } else {
            robot.extension.setPower(extendoPower);
        }
    }

    public void setExtendoTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_EXTENDO_EXTENSION), 0);
        extendoPIDF.setSetPoint(this.target);
    }

    public void stopSlide() {
        robot.extension.setPower(0);
    }

    public void setPivot(IntakePivotState intakePivotState) {
        switch (intakePivotState) {
            case READY_INTAKE:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_READY_PICKUP_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_READY_PICKUP_POS);
                break;
            case TRANSFER:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_TRANSFER_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_TRANSFER_POS);
                break;
            case MIDDLE_HOLD:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_HOLD_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_HOLD_POS);
                break;
            case INTAKE:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_PICKUP_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_PICKUP_POS);
                break;
        }

        Intake.intakePivotState = intakePivotState;
    }

    public void setClawState(ClawState clawState) {
        this.clawState = clawState;
        setClawOpen(clawOpen);
    }

    public void setClawOpen(boolean open) {
        switch (clawState) {
            case INNER:
                if (open) {
                    robot.intakeClaw.setPosition(INTAKE_CLAW_INNER_OPEN_POS);
                } else {
                    robot.intakeClaw.setPosition(INTAKE_CLAW_INNER_CLOSE_POS);
                }
                break;
            case OUTER:
                if (open) {
                    robot.intakeClaw.setPosition(INTAKE_CLAW_OUTER_OPEN_POS);
                } else {
                    robot.intakeClaw.setPosition(INTAKE_CLAW_OUTER_CLOSE_POS);
                }
                break;
        }

        this.clawOpen = open;
    }

    public void setWrist(WristState wristState) {
        switch (wristState) {
            case TRANSFER:
                switch (clawState) {
                    case OUTER:
                        robot.wrist.setPosition(WRIST_OUTER_TRANSFER_POS);
                        break;
                    case INNER:
                        robot.wrist.setPosition(WRIST_INNER_TRANSFER_POS);
                        break;
                }
                break;
            case ROTATED:
                robot.wrist.setPosition(WRIST_POSITIONS[Math.max(Math.min(wristIndex, 4), 0)]);
                break;
            case INTAKE:
                robot.wrist.setPosition(WRIST_INTAKE_POS);
                break;
        }
        Intake.wristState = wristState;
    }

    public void setWristIndex(int index) {
        this.wristIndex = index;
    }

    public void setTrayOpen(boolean open) {
        if (open) {
            robot.trayServo.setPosition(TRAY_OPEN_POS);
        } else {
            robot.trayServo.setPosition(TRAY_CLOSE_POS);
        }
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
        autoUpdateExtendo();
    }
}
