package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

public class depositSafeRetracted extends CommandBase {
    Deposit deposit;
    private boolean finished = false;
    private ElapsedTime timer = new ElapsedTime();

    public depositSafeRetracted(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setSlideTarget(SLIDES_PIVOT_READY_EXTENSION);
        deposit.setPivot(Deposit.DepositPivotState.MIDDLE_HOLD);
        timer.reset();
    }

    @Override
    public void execute() {
        if ((timer.milliseconds() > 400) && !finished) {
            finished = true;
            deposit.setSlideTarget(0);
        }
    }

    @Override
    public boolean isFinished() {
        return deposit.slidesReached && finished;
    }
}
