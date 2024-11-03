package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.subsystem.Intake.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class setIntake extends CommandBase {
    Intake intake;
    Intake.IntakePivotState state;
    // Timer to give claw time to close/open
    ElapsedTime timer = new ElapsedTime();
    private boolean finished = false;

    public setIntake(Intake intake, IntakePivotState state) {
        this.intake = intake;
        this.state = state;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        switch (state) {
            case READY_INTAKE:
                intake.setClawOpen(true);
                intake.setWrist(Intake.WristState.INTAKE);
                intake.setTrayOpen(true);
                break;
            case INTAKE:
                intake.setClawOpen(true);
                intake.setWrist(Intake.WristState.INTAKE);
                intake.setTrayOpen(true);
                break;
            case TRANSFER:
                intake.setClawOpen(false);
                intake.setWrist(WristState.TRANSFER);
                intake.setTrayOpen(true);
                break;
            case MIDDLE_HOLD:
                intake.setClawOpen(true);
                intake.setWrist(WristState.TRANSFER);
                intake.setTrayOpen(false);
                break;
        }
        timer.reset();
    }
    @Override
    public void execute() {
        if ((timer.milliseconds() > 350) && !finished) {
            finished = true;
            intake.setPivot(state);
        }
    }

    @Override
    public boolean isFinished() {
        return (timer.milliseconds() > 700) && finished;
    }
}
