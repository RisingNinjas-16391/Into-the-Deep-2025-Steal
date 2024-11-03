package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.SLIDES_PIVOT_READY_EXTENSION;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class setDepositScoring extends SequentialCommandGroup {
    public setDepositScoring(Deposit deposit, double target, Deposit.DepositPivotState state) {
        addRequirements(deposit);
        addCommands(
                new setDepositSlidesScoring(deposit, target),
                new setDeposit(deposit, state));
    }
}
