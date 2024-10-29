package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class realTransfer extends SequentialCommandGroup {
    public realTransfer(Deposit deposit, Intake intake) {
        addCommands(
                new transfer(deposit, intake),
                new transferToDeposit(deposit, intake)
        );
        addRequirements(deposit, intake);
    }
}
