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
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.commands.FTCLibAction;
import org.firstinspires.ftc.teamcode.subsystem.commands.depositSafeRetracted;
import org.firstinspires.ftc.teamcode.subsystem.commands.setDepositScoring;

@Config
@Autonomous
public class BlueObservation extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    Action moveToSpecimen;
    Action moveBack;
    Action pushSamples;

    @Override
    public void initialize() {
        robot.drive.pose = new Pose2d(-8, 61.75, Math.toRadians(270));;

        moveToSpecimen = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-8, 37.75))
                .waitSeconds(2.0)
                .build();

        moveBack = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-8, 39.75))
                .waitSeconds(1.0)

                .build();

        pushSamples = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-8, 39.75))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-34.5, 10.1), Math.toRadians(273))
                .waitSeconds(0.5)
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(-46, 10.1))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-46, 50.1))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-46, 10.1))
                .strafeToConstantHeading(new Vector2d(-53, 10.1))
                .strafeToConstantHeading(new Vector2d(-53, 50.1))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-53, 10.1))
                .strafeToConstantHeading(new Vector2d(-61, 10.1))
                .strafeToConstantHeading(new Vector2d(-61, 54.5))

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
                pushSamples
            )
        );

        super.run();
    }
}