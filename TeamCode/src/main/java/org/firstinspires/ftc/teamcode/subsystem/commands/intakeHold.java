package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.*;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class intakeHold extends CommandBase {
    Intake intake;

    public intakeHold(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.openClaw();
        intake.setPivotServo(INTAKE_PIVOT_HOLD_POS);
        intakePivotState = ExtendoState.IntakePivotState.MIDDLE_HOLD;
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
