package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class intakeTransferReady extends CommandBase {
    Intake intake;

    public intakeTransferReady(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
//        Intake.intakeState = Intake.IntakeState.FULL_TRANSFER;
        intake.closeClaw();
        intake.setWrist(WRIST_TRANSFER_POS);
        intake.setPivotServo(INTAKE_PIVOT_TRANSFER_POS);
        intake.setExtendoTarget(0);
    }

    @Override
    public boolean isFinished() {
        return (intake.extendoRetracted);
    }
}
