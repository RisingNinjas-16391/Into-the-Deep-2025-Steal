package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

import static org.firstinspires.ftc.teamcode.subsystem.Deposit.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

public class depositTransferReady extends CommandBase {
    Deposit deposit;

    public depositTransferReady(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        if (intakeState.equals(Intake.ExtendoState.FULL_EXTENSION) || intakeState.equals(Intake.ExtendoState.HALF_EXTENSION)) {
            deposit.setSlideTarget(SLIDES_PIVOT_READY_EXTENSION);
            deposit.pivotTransferPos();
        }
        deposit.setSlideTarget(0);
        deposit.openClaw();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        deposit.stopSlide();
    }
}
