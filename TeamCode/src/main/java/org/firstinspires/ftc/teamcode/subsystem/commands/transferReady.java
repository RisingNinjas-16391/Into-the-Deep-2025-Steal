package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class transferReady extends SequentialCommandGroup {
    public transferReady(Deposit deposit, Intake intake) {
        addCommands(
                new setIntake(intake, Intake.IntakePivotState.TRANSFER),
                new InstantCommand(() -> intake.setExtendoTarget(0)),
                new depositSafeRetracted(deposit)
        );
        addRequirements(deposit, intake);
    }
}
