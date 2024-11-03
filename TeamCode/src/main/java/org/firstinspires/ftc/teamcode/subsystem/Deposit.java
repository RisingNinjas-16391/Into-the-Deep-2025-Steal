package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Deposit extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private static final PIDFController slidePIDF = new PIDFController(0.009,0,0.0002, 0.00016);

    // Between open and closed
    public boolean clawOpen;

    public double target;

    public boolean slidesReached;

    // Between retracted and extended
    public boolean slidesRetracted;

    public enum SlideState {
        SCORING,
        TRANSFER,
        PIVOT_READY,
        SPECIMEN_INTAKE
    }

    public SlideState slideState;

    public enum DepositPivotState {
        SCORING,
        SPECIMEN_SCORING,
        TRANSFER,
        MIDDLE_HOLD,
        INTAKE
    }

    public DepositPivotState depositPivotState;

    public void init() {
        slidePIDF.setTolerance(10, 10);
        setSlideTarget(0);

        // OpMode specific initializations
        if (opModeType.equals(OpModeType.AUTO)) {
            setPivot(DepositPivotState.SPECIMEN_SCORING);
            setClawOpen(false);

        } else if (opModeType.equals(OpModeType.TELEOP)) {
            setPivot(DepositPivotState.SPECIMEN_SCORING);
            setClawOpen(false);
        }
    }

    public void setSlideTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_SLIDES_EXTENSION), 0);
        slidePIDF.setSetPoint(target);
    }

    public void stopSlide() {
        robot.liftTop.setPower(0);
        robot.liftBottom.setPower(0);
    }

    public void autoUpdateSlides() {
        double power = slidePIDF.calculate(robot.liftEncoder.getPosition(), target);
        robot.liftTop.setPower(power);
        robot.liftBottom.setPower(power);

        slidesReached = slidePIDF.atSetPoint();
        slidesRetracted = (target <= 0) && slidesReached;
    }

    public void setClawOpen(boolean open) {
        if (open) {
            robot.depositClaw.setPosition(DEPOSIT_CLAW_OUTSIDE_OPEN_POS);
        } else {
            robot.depositClaw.setPosition(DEPOSIT_CLAW_OUTSIDE_CLOSE_POS);
        }

        this.clawOpen = open;
    }

    public void setPivot(DepositPivotState depositPivotState) {
        switch (depositPivotState) {
            case SCORING:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_SCORING_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_SCORING_POS);
                break;
            case SPECIMEN_SCORING:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_SCORING_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_SCORING_POS);
                break;
            case TRANSFER:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_TRANSFER_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_TRANSFER_POS);
                break;
            case INTAKE:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS);
                break;
            case MIDDLE_HOLD:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_MIDDLE_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_MIDDLE_POS);
                break;
        }

        this.depositPivotState = depositPivotState;
    }

    @Override
    public void periodic() {
        autoUpdateSlides();
    }
}
