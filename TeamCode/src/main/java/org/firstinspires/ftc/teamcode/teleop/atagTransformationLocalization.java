///* Copyright (c) 2024 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
//import org.firstinspires.ftc.teamcode.robot.subsystems.Transform3d;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
///**
// * This OpMode demonstrates using a camera with the FTC SDK and the Transform3d class.
// *
// * The OpMode initializes a webcam, performs transformations, and displays the results in telemetry.
// * The Transform3d class is used to handle 3D transformations and operations.
// *
// * Use Android Studio to copy this class and paste it into your team's code folder with a new name.
// */
//@TeleOp(name = "Transform3d Camera ", group = "atag")
//public class atagTransformationLocalization extends LinearOpMode {
//
//    private OpenCvCamera webcam;
//
//    @Override
//    public void runOpMode() {
//        // Initialize the camera
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        // Open the camera device
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                // Start streaming with a resolution of 640x480
//                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Error", "Error Code: %d", errorCode);
//            }
//        });
//
//        // Example: Create two Transform3d instances to demonstrate operations
//        VectorF translation1 = new VectorF(1.0f, 2.0f, 0.0f);
//        Quaternion rotation1 = new Quaternion(1, 0, 0, 0, System.nanoTime());
//
//        VectorF translation2 = new VectorF(0.5f, 1.5f, 0.0f);
//        Quaternion rotation2 = new Quaternion(0.707f, 0, 0.707f, 0, System.nanoTime());
//
//        Transform3d transform1 = new Transform3d(translation1, rotation1);
//        Transform3d transform2 = new Transform3d(translation2, rotation2);
//
//        // Wait for the start button to be pressed
//        telemetry.addData(">", "Touch Play to start OpMode");
//        telemetry.update();
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Use the getter methods to access translation and rotation
//            telemetry.addData("Transform 1", "Translation: %s, Rotation: %s", transform1.translation, transform1.rotation);
//            telemetry.addData("Transform 2", "Translation: %s, Rotation: %s", transform2.translation, transform2.rotation);
//
//            // Perform addition of transforms
//            Transform3d resultTransform = transform1.plus(transform2);
//            telemetry.addData("Resulting Transform (Addition)", "Translation: %s, Rotation: %s", resultTransform.translation, resultTransform.rotation);
//
//            // Push telemetry to the Driver Station.
//            telemetry.update();
//
//            // Save CPU resources; can resume streaming when needed.
//            if (gamepad1.dpad_down) {
//                webcam.stopStreaming();
//            } else if (gamepad1.dpad_up) {
//                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            // Share the CPU.
//            sleep(20);
//        }
//
//        // Stop streaming when the OpMode ends
//        webcam.stopStreaming();
//    }
//}
