package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.subsystem.Intake.IntakePivotState;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.WristState;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class manualMoveWrist extends CommandBase {
    Intake intake;
    private boolean increase;

    public manualMoveWrist(Intake intake, boolean increase) {
        this.intake = intake;
        this.increase = increase;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (increase) {
            intake.setWristIndex(intake.wristIndex - 1);
        }
        else {
            intake.setWristIndex(intake.wristIndex + 1);
        }

        intake.setWrist(Intake.WristState.ROTATED);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
