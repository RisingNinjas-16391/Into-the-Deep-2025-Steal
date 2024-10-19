package org.firstinspires.ftc.teamcode.roadrunner.tuning.otos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;
@TeleOp
public class OTOSAngularScalar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0,0,0));

        double radsTurned = 0;
        Rotation2d lastHeading = Rotation2d.fromDouble(0);

        telemetry.addLine("OTOS Angular Scalar Tuner");
        telemetry.addLine("Press START, then rotate the robot on the ground 10 times (3600 degrees).");
        telemetry.addLine("Then copy the scalar into SparkFunOTOSDrive.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            radsTurned += drive.pose.heading.minus(lastHeading);
            lastHeading = drive.pose.heading;
            telemetry.addData("Uncorrected Degrees Turned", Math.toDegrees(radsTurned));
            telemetry.addData("Calculated Angular Scalar", Math.abs(3600 / Math.toDegrees(radsTurned)));
            telemetry.update();

            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger = gamepad1.left_trigger;

            if (rightTrigger > 0  && leftTrigger == 0) {
                drive.leftFront.setPower(rightTrigger);
                drive.leftBack.setPower(rightTrigger);
                drive.rightFront.setPower(-rightTrigger);
                drive.rightBack.setPower(-rightTrigger);
            }

            else if (leftTrigger > 0 && rightTrigger == 0) {
                drive.leftFront.setPower(-leftTrigger);
                drive.leftBack.setPower(-leftTrigger);
                drive.rightFront.setPower(leftTrigger);
                drive.rightBack.setPower(leftTrigger);
            }

            else {
                drive.leftFront.setPower(0);
                drive.leftBack.setPower(0);
                drive.rightFront.setPower(0);
                drive.rightBack.setPower(0);
            }
        }
    }
}
