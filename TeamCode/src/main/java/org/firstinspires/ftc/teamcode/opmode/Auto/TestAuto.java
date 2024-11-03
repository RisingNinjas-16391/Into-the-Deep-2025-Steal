package org.firstinspires.ftc.teamcode.opmode.Auto;
import static org.firstinspires.ftc.teamcode.hardware.Globals.HIGH_SPECIMEN_HEIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.commands.*;

@Config
@Autonomous
public class TestAuto extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    Action moveToSpecimen;
    Action moveBack;
    Action pushSamples;
    Action park;

    @Override
    public void initialize() {
        moveToSpecimen = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(8, 37.75))
                .waitSeconds(2.0)
                .build();

        moveBack = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(8, 39.75))
                .waitSeconds(1.0)

                .build();

        pushSamples = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(8, 39.75))
                .splineToConstantHeading(new Vector2d(34.5, 10.1), Math.toRadians(273))
                .strafeToConstantHeading(new Vector2d(46, 12.1))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(46, 60.1))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(46, 12.1))
                .strafeToConstantHeading(new Vector2d(53, 12.1))
                .strafeToConstantHeading(new Vector2d(53, 53.1))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(53, 12.1))
                .strafeToConstantHeading(new Vector2d(61, 12.1))
                .strafeToConstantHeading(new Vector2d(61, 48.5))

                .build();

        park = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(57, 48.5))
                .splineToLinearHeading(new Pose2d(28.2, 15, Math.toRadians(180)), Math.toRadians(225))
                .waitSeconds(1.0)
                .strafeToConstantHeading(new Vector2d(26.2, 15))
                .build();

        robot.init(hardwareMap);
    }

    @Override
    public void run() {
        Actions.runBlocking(
            new SequentialAction(
                new ParallelAction(
                    new FTCLibAction(new setDepositScoring(robot.deposit, HIGH_SPECIMEN_HEIGHT, Deposit.DepositPivotState.SPECIMEN_SCORING)),
                    moveToSpecimen
                ),
                new ParallelAction(
                    new FTCLibAction(new depositSafeRetracted(robot.deposit)),
                    moveBack
                ),
                pushSamples,
                new ParallelAction(
                        new FTCLibAction(new setDepositScoring(robot.deposit, HIGH_SPECIMEN_HEIGHT, Deposit.DepositPivotState.SPECIMEN_SCORING)),
                        park
                )
            )
        );

        super.run();
    }
}