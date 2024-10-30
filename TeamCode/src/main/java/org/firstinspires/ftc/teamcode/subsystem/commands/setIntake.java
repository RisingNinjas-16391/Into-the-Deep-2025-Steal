package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class setIntake extends CommandBase {
    Intake intake;
    ExtendoState.IntakePivotState state;

    public setIntake(Intake intake, ExtendoState.IntakePivotState state) {
        this.intake = intake;
        this.state = state;

        addRequirements(intake);
    }

    @Override
    public void initialize() {

        switch (state) {
            case READY_INTAKE:
                intake.setWristIntake();
                intake.setPivotServo(INTAKE_PIVOT_READY_PICKUP_POS);
                intake.openClaw();
                break;

            case INTAKE:
                intake.setWristIntake();
                intake.setPivotServo(INTAKE_PIVOT_PICKUP_POS);
                intake.openClaw();
                break;

            case TRANSFER:
                intake.setPivotServo(INTAKE_PIVOT_HOLD_POS);
                intake.setWristTransfer();
                intake.openClaw();
                break;

            case MIDDLE_HOLD:
                intake.openClaw();
                intake.setPivotServo(INTAKE_PIVOT_HOLD_POS);
                intake.setWristTransfer();
                break;
        }

//        intake.setExtendoTarget(0);
        // intake.extendoRetracted will change the intake state automatically
    }

    @Override
    public boolean isFinished() {
        return (intake.extendoRetracted);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopSlide();
    }
}
