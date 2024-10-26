package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class depositHighBasket extends CommandBase {
    Deposit deposit;

    public depositHighBasket(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setSlideTarget(BACKDROP_INCREMENTAL_HEIGHT);
        deposit.openClaw();
        deposit.wristIndex = 3;
        deposit.moveWrist();
//        deposit.setArmTransfer(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
