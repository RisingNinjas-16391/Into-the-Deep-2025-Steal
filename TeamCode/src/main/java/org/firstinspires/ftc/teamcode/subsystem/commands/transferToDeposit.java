package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class transferToDeposit extends SequentialCommandGroup {
    public transferToDeposit(Deposit deposit, Intake intake) {
        addCommands(
                new setIntake(intake, Intake.IntakePivotState.MIDDLE_HOLD),
                new setDeposit(deposit, Deposit.DepositPivotState.TRANSFER)
        );
        addRequirements(deposit, intake);
    }
}
