package org.firstinspires.ftc.teamcode.opmode.Auto;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.commands.FTCLibAction;
import org.firstinspires.ftc.teamcode.subsystem.commands.depositTransferReady;

@Config
@Autonomous
public class TestAuto extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private Action trajectoryAction;

    @Override
    public void initialize() {
        trajectoryAction = robot.drive.actionBuilder(robot.drive.pose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3)
                .build();

//        drive.pose = new Pose2d(AprilTagX, AprilTagY, drive.pose.heading);

        robot.init(hardwareMap);
    }

    @Override
    public void run() {
        Actions.runBlocking(
                new FTCLibAction(new depositTransferReady(robot.deposit))
        );
    }
}