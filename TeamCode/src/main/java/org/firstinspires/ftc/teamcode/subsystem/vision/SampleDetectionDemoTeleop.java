package org.firstinspires.ftc.teamcode.subsystem.vision;

import static org.firstinspires.ftc.teamcode.subsystem.vision.SampleDetectionPipelinePNP.calculateDistance;
import static org.firstinspires.ftc.teamcode.subsystem.vision.SampleDetectionPipelinePNP.pointsList;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvInternalCamera;

import com.acmerobotics.dashboard.FtcDashboard;

import java.util.ArrayList;

@TeleOp
public class SampleDetectionDemoTeleop extends LinearOpMode {
    OpenCvCamera webcam;
    SampleDetectionPipelinePNP pipeline;

    @Override
    public void runOpMode() {
        // Get the camera monitor view ID

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        pipeline = new SampleDetectionPipelinePNP();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480); // Set your preferred resolution
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera failed to open");
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ArrayList<SampleDetectionPipelinePNP.AnalyzedStone> objectCount = pipeline.getDetectedStones();

            telemetry.addData("pointsList Distance", calculateDistance(pointsList.toString()));
            telemetry.addData("Objects Detected", objectCount);
            telemetry.update();
        }
        webcam.stopStreaming();
    }
}