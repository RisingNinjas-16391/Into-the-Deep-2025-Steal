package org.firstinspires.ftc.teamcode.opmode.Auto;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.commands.depositSafeRetracted;

@Config
@Autonomous
public class RightAuto extends OpMode {
    private final Robot robot = Robot.getInstance();
    public static int index = 0;
    public static double motorSpeeds = 0.3;
    public static int stopTimer = 2200;

    public ElapsedTime timer;

    @Override
    public void init() {
        opModeType = OpModeType.AUTO;
        startingPose = new Pose2d(0, 0, 0);

        CommandScheduler.getInstance().enable();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        CommandScheduler.getInstance().registerSubsystem(robot.deposit, robot.intake);

    }

    @Override
    public void loop() {
        if (timer == null) {

            timer = new ElapsedTime();
            robot.deposit.setSlideTarget(HIGH_SPECIMEN_HEIGHT);

            robot.drive.leftBack.setPower(motorSpeeds);
            robot.drive.leftFront.setPower(motorSpeeds);
            robot.drive.rightBack.setPower(motorSpeeds);
            robot.drive.rightFront.setPower(motorSpeeds);

            robot.colorSensor.enableLed(true);
            if (index == 0) {
                index = 1;
            }
        }

        telemetry.addData("index", index);
        telemetry.addData("liftBottom power", robot.liftBottom.getPower());
        telemetry.addData("liftTop power", robot.liftTop.getPower());
        telemetry.update();

        if (timer.milliseconds() >= stopTimer && index == 1) {
            robot.drive.leftBack.setPower(0);
            robot.drive.leftFront.setPower(0);
            robot.drive.rightBack.setPower(0);
            robot.drive.rightFront.setPower(0);

            index = 2;
        }

        if (timer.milliseconds() >= (1000 + stopTimer) && index == 2) {
            robot.deposit.setSlideTarget(HIGH_SPECIMEN_ATTACH_HEIGHT);

            index = 3;
        }

        if (timer.milliseconds() >= (1500 + stopTimer) && index == 3) {
            robot.deposit.setClawOpen(true);

            sleep(500);

            robot.drive.leftBack.setPower(-motorSpeeds);
            robot.drive.leftFront.setPower(-motorSpeeds);
            robot.drive.rightBack.setPower(-motorSpeeds);
            robot.drive.rightFront.setPower(-motorSpeeds);

            timer.reset();
            index = 4;
        }

        if (timer.milliseconds() >= (stopTimer - 300) && index == 4) {
            robot.drive.leftBack.setPower(0);
            robot.drive.leftFront.setPower(0);
            robot.drive.rightBack.setPower(0);
            robot.drive.rightFront.setPower(0);

            CommandScheduler.getInstance().schedule(new depositSafeRetracted(robot.deposit));

            robot.drive.leftBack.setPower(-motorSpeeds);
            robot.drive.leftFront.setPower(+motorSpeeds);
            robot.drive.rightBack.setPower(+motorSpeeds);
            robot.drive.rightFront.setPower(-motorSpeeds);

            timer.reset();
            index = 5;
        }

        if (timer.milliseconds() >= stopTimer * 2 && index == 5) {
            robot.drive.leftBack.setPower(0);
            robot.drive.leftFront.setPower(0);
            robot.drive.rightBack.setPower(0);
            robot.drive.rightFront.setPower(0);
            index = 6;
        }

        CommandScheduler.getInstance().run();
    }
}