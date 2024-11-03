package org.firstinspires.ftc.teamcode.subsystem.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

public class FTCLibAction implements Action {
    private final CommandBase command;
    private boolean initialized = false;
    public FTCLibAction(CommandBase command) {
        this.command = command;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        final boolean initialized = this.initialized;
        if (!initialized) {
            CommandScheduler.getInstance().schedule(command);
            this.initialized = true;
        }

        final boolean finished = initialized && !CommandScheduler.getInstance().isScheduled(command);
        if (finished) {
            this.initialized = false;
        }
        return finished;
    }
}
