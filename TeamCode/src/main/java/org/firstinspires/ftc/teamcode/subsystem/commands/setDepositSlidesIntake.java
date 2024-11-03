package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class setDepositSlidesIntake extends CommandBase {
    Deposit deposit;
    private boolean finished = false;
    ElapsedTime timer = new ElapsedTime();

    public setDepositSlidesIntake(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setClawOpen(false);
        deposit.setSlideTarget(SLIDES_PIVOT_READY_EXTENSION + 50);

        deposit.setPivot(Deposit.DepositPivotState.INTAKE);
    }

    @Override
    public void execute() {
        if (deposit.slidesReached && !finished) {
            deposit.setSlideTarget(0);
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (deposit.slidesReached && finished) {
            new depositSafeRetracted(deposit);
        }
        return deposit.slidesReached && finished;
    }
}
