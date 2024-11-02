package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.subsystem.Intake.IntakePivotState;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.WristState;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class setDeposit extends CommandBase {
    Deposit deposit;
    Deposit.DepositPivotState state;
    // Timer to give claw time to close/open
    ElapsedTime timer = new ElapsedTime();
    private boolean finished = false;

    public setDeposit(Deposit deposit, Deposit.DepositPivotState state) {
        this.deposit = deposit;
        this.state = state;

        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        switch (state) {
            case MIDDLE_HOLD:
                deposit.setClawOpen(true);
                break;
            case SCORING:
                deposit.setClawOpen(false);
                break;
            case TRANSFER:
                deposit.setClawOpen(true);
                break;
        }
        timer.reset();
    }

    @Override
    public void execute() {
        if ((timer.milliseconds() > 450) && !finished) {
            finished = true;
            deposit.setPivot(state);
        }
    }

    @Override
    public boolean isFinished() {
        return (timer.milliseconds() > 500) && finished;
    }
}
