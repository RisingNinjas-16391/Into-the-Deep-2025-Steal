package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class transferToDeposit extends SequentialCommandGroup {
    public transferToDeposit(Deposit deposit, Intake intake) {
        addCommands(
                new intakeHold(intake),
                new depositTransferReady(deposit)
        );
        addRequirements(deposit, intake);
    }
}
