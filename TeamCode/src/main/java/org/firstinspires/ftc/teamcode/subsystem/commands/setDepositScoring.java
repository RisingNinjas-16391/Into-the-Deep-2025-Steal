package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.SLIDES_PIVOT_READY_EXTENSION;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class setDepositScoring extends SequentialCommandGroup {
    Deposit deposit;

    ElapsedTime timer = new ElapsedTime();

    public setDepositScoring(Deposit deposit, double target) {
        this.deposit = deposit;
        addCommands(new setDepositSlidesScoring(deposit, target),
                    new InstantCommand(() -> deposit.setPivot(Deposit.DepositPivotState.SCORING)),
                    new setDepositSlidesScoring(deposit, target)); // Does this twice just in case the first one is used to move the slides up to pivot-able position
        addRequirements(deposit);
    }
}
