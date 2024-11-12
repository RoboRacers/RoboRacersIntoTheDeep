package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Sample Other Team OpMode", group = "Vision")
public class SampleOtherTeamOpMode extends LinearOpMode {
    OpenCvCamera camera;
    SampleOtherTeamCode pipeline;

    Servo rotateClaw;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {

        claw = hardwareMap.get(Servo.class,"claw");
        rotateClaw = hardwareMap.get(Servo.class,"rotateClaw");

        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize the pipeline
        pipeline = new SampleOtherTeamCode();
        camera.setPipeline(pipeline);

        // Open the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Optionally start streaming the camera feed to the viewport
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // You can process the detected stones here if needed

//            claw.setPosition(gamepad1.left_stick_x);
//            rotateClaw.setPosition(gamepad1.right_stick_y);
            telemetry.addData("Detected Stones", pipeline.getDetectedStones().size());

            if (gamepad1.triangle){
                pipeline.targetColor= "Blue";
            }

            else if (gamepad1.cross){
                pipeline.targetColor= "Red";
            }
            else if (gamepad1.square){
                pipeline.targetColor= "Yellow";
            }





            double targetAngle = pipeline.targetAngle;

            telemetry.addData("Target angle", targetAngle);



            telemetry.addData("Claw Value", claw.getPosition());
            telemetry.addData("Rotate Value", rotateClaw.getPosition());

            telemetry.update();

            // Add any other logic or telemetry you want here

            // Sleep to reduce CPU usage
            sleep(100);
        }

        // Stop the camera when the OpMode ends
        camera.stopStreaming();
    }
}