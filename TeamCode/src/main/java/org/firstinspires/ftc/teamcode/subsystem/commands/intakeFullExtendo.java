package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.*;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class intakeFullExtendo extends CommandBase {
    Intake intake;

    public intakeFullExtendo(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setWristIntake();
        intake.openClaw();
        intake.setPivotServo(INTAKE_PIVOT_READY_PICKUP_POS);
        intake.setExtendoTarget(MAX_EXTENDO_EXTENSION);
        extendoState = ExtendoState.FULL_EXTENSION;
}

    @Override
    public boolean isFinished() {
        return (!intake.extendoRetracted);
    }
}
