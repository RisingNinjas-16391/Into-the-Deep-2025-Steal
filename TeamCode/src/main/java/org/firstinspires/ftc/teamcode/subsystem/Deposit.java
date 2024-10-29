package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Deposit extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private static final PIDFController slidePIDF = new PIDFController(0,0,0, 0);

    private double target;

    // Between retracted and extended
    public boolean slidesRetracted;

    // Between open and closed
    public boolean clawOpen;

    public enum SlideState {
        SCORING,
        TRANSFER,
        PIVOT_READY;
        public enum DepositPivotState {
            SCORING,
            TRANSFER,
            MIDDLE_HOLD
        }
    }

    public static Intake.ExtendoState intakeState;
    public static Intake.ExtendoState.IntakePivotState extendoPivotState;

    // Default will reset deposit to transfer position (unpowered claw servos depending on auto vs tele-op)
    public void init() {
        slidePIDF.setTolerance(10, 10);
        setSlideTarget(0);
    }

    public void initAuto() {
        closeClaw();
    }

    public void initTeleOp() {

    }

    public void setSlideTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_SLIDES_EXTENSION), 0);
        slidePIDF.setSetPoint(target);
    }

    public void stopSlide() {
        robot.liftTop.setPower(0);
        robot.liftBottom.setPower(0);
    }

    public void autoSetSlidePower() {
        double power = slidePIDF.calculate(robot.liftEncoder.getPosition(), target);
        robot.liftTop.setPower(power);
        robot.liftBottom.setPower(power);

        // Slides are only retracted once stopped and at a target of 0
        slidesRetracted = ((target <= 0)) && (this.reached());
    }

    // Returns if slides have reached the target
    public boolean reached() {
        return (slidePIDF.atSetPoint());
    }

    public void openClaw() {
        robot.depositClaw.setPosition(DEPOSIT_CLAW_OPEN_POS);
        this.clawOpen = true;
    }

    public void closeClaw() {
        robot.depositClaw.setPosition(DEPOSIT_CLAW_CLOSE_POS);
        this.clawOpen = false;
    }

    public void pivotTransferPos() {
        robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_TRANSFER_POS);
        robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_TRANSFER_POS);
    }

    @Override
    public void periodic() {
        autoSetSlidePower();
    }
}
