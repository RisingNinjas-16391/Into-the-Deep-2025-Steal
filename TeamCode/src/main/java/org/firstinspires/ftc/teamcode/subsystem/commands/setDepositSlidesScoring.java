package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class setDepositSlidesScoring extends CommandBase {
    Deposit deposit;
    private double target;
    private boolean finished = false;
    ElapsedTime timer = new ElapsedTime();

    public setDepositSlidesScoring(Deposit deposit, double target) {
        this.deposit = deposit;
        this.target = target;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setClawOpen(false);
        if (deposit.target >= SLIDES_PIVOT_READY_EXTENSION) {
            deposit.setSlideTarget(target);
            finished = true;
        } else {
            deposit.setSlideTarget(SLIDES_PIVOT_READY_EXTENSION);
        }
    }

    @Override
    public void execute() {
        if (deposit.slidesReached && !finished) {
            deposit.setSlideTarget(target);
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return deposit.slidesReached && finished;
    }
}
