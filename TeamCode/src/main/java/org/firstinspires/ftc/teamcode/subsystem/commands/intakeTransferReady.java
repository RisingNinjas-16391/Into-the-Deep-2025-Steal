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
        intake.closeClaw();
        intake.setWristTransfer();
        intake.setPivotServo(INTAKE_PIVOT_TRANSFER_POS);
        intake.setExtendoTarget(0);
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
