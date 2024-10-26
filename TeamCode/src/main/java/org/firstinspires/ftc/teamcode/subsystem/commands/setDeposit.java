package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class setDeposit extends CommandBase {
    Deposit deposit;
    double target;

    public setDeposit(Deposit deposit, double target) {
        this.deposit = deposit;
        this.target = target;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setSlideTarget(target);
        deposit.pivotTransferPos();
        deposit.openClaw();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
