package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class transfer extends ParallelCommandGroup {
    public transfer (Deposit deposit, Intake intake) {
        addCommands(
                new intakeTransferReady(intake),
                new depositTransferReady(deposit)
        );
        addRequirements(deposit, intake);
    }
}
