package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class setDepositSlidesIntake extends CommandBase {
    Deposit deposit;
    private boolean finished = false;
    private boolean earlyFinished = false;
    ElapsedTime timer = new ElapsedTime();

    public setDepositSlidesIntake(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setSlideTarget(SLIDES_PIVOT_READY_EXTENSION + 50);
        deposit.setClawOpen(false);
    }

    @Override
    public void execute() {
        if (deposit.slidesReached && !earlyFinished && !deposit.slidesRetracted) {
            deposit.setPivot(Deposit.DepositPivotState.INTAKE);
            earlyFinished = true;
            timer.reset();
        }
        if (timer.milliseconds() > 300 && earlyFinished) {
            deposit.setClawOpen(true);
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return deposit.slidesReached && finished;
    }

    @Override
    public void end(boolean interruptable) {
        deposit.setSlideTarget(0);
    }
}
