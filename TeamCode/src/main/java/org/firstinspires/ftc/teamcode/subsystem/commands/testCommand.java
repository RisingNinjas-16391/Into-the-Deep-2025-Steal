package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.subsystem.Intake.IntakePivotState;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.WristState;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class testCommand extends CommandBase {
    Deposit deposit;

    public testCommand(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);
    }

    @Override
    public void execute() {
        deposit.setClawOpen(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
