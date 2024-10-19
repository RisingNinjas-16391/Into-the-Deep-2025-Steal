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

    public IntakeState intakeState;
    private static final PIDFController extendoPIDF = new PIDFController(0,0,0, 0);

    // Loop number for when distance sensor was polled
    private int loopNumber = 0;

    public void init() {
        setExtendoTarget(0);
    }

    public void autoExtendoSetPower() {
        double extendoPower = extendoPIDF.calculate(robot.liftEncoder.getPosition(), target);
        robot.liftLeft.setPower(extendoPower);
        robot.liftRight.setPower(-extendoPower);

        // Extendo is only retracted it has reached a target of 0
        extendoRetracted = ((target <= 0)) && (this.reached());
    }

    // Returns if extendo has reached the target
    public boolean reached() {
        return (extendoPIDF.atSetPoint());
    }

    public void setExtendoTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_EXTENDO_EXTENSION), 0);
        extendoPIDF.setSetPoint(this.target);
    }

//    public void setPitchingIntake(int stackHeight) {
//        if (stackHeight != this.stackHeight) {
//            // Limit integers to 0-5
//            this.stackHeight = Math.max(Math.min(stackHeight, 5), 0);
//            robot.pitchingIntake.setPosition(STACK_HEIGHTS[this.stackHeight]);
//        }
//    }
//
//    public void setTray(boolean trayTransfer) {
//        if (trayTransfer != this.trayTransfer) {
//            robot.tray.setPosition(trayTransfer ? TRAY_TRANSFER : TRAY_INTAKE);
//            this.trayTransfer = trayTransfer;
//        }
//    }

    public void sampleDetected() {
        if (intakeState == IntakeState.INTAKE) {
            if (robot.colorSensor.getDistance(DistanceUnit.CM) < 5) {
                assert true;
            }
        }
    }



    @Override
    public void periodic() {
        autoExtendoSetPower();
    }
}
