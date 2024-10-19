package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.*;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class intakeNoExtendo extends CommandBase {
    Intake intake;

    public intakeNoExtendo(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intakeState = Intake.IntakeState.INTAKE;
        intake.openClaw();
        intake.setWrist(WRIST_INTAKE_POS);
        intake.setPivotServo(PIVOT_INTAKE_POS);
        intake.setExtendoTarget(0);
    }

    @Override
    public boolean isFinished() {
        return (intake.extendoRetracted);
    }
}
