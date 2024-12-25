package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.vision.CombinedHSVAndAnglePipeline;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.CombinedHSVAndAnglePipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "HSV Allign AutoOp", group = "Vision")
public class AutoMoveHSV extends LinearOpMode {

    MecanumDrive drive;
    OpenCvCamera camera;
    CombinedHSVAndAnglePipeline pipeline;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        runtime.reset();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CombinedHSVAndAnglePipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
//        telemetry.addData("object angle: ",pipeline.angle);
//        telemetry.update();

        waitForStart();

        double xCenter = pipeline.rotatedRect.center.x;
        double yCenter = pipeline.rotatedRect.center.y;

        double xCamera = 320;
        double yCamera = 240;

        double xErrorPixels = xCenter - xCamera;
        double yErrorPixels = yCamera - yCenter;

        double length;

        double height = pipeline.rotatedRect.size.height;
        double width = pipeline.rotatedRect.size.width;

        length = Math.max(height, width);
        telemetry.addData("Length in pixels", length);

        double xErrorInches = xErrorPixels * (3.5 / length);
        double yErrorInches = yErrorPixels * (3.5 / length);
        telemetry.addData("X Error in Inches", xErrorInches);
        telemetry.addData("Y Error in Inches", yErrorInches);


        Action trajectory = drive.actionBuilder(new Pose2d(0, 0,0))
                .splineToLinearHeading(new Pose2d(-(yErrorInches*3), -(xErrorInches*2), Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(1)
                .build();
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        trajectory
                ),
                telemetryPacket -> {
                    telemetry.update();
                    return opModeIsActive();
                }
        ));

        telemetry.update();

    }
}