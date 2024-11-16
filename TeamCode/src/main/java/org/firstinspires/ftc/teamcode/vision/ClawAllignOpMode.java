package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.vision.SampleOtherTeamCode;

@TeleOp(name = "Sample Claw Allign", group = "Vision")
public class ClawAllignOpMode extends LinearOpMode {
    OpenCvCamera camera;
    SampleOtherTeamCode pipeline;

    CRServo rotateClaw;
   // Servo claw;

    // Constants for controlling the servo
    private static final double ROTATE_KP = 0.01; // Adjust this to fine-tune control
    private static final double CLAW_OPEN_POSITION = 0.0; // Adjust based on your claw's open position
    private static final double CLAW_CLOSED_POSITION = 1.0; // Adjust based on your claw's closed position
    private static final double ROTATE_MIN_POSITION = 0.0;
    private static final double ROTATE_MAX_POSITION = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

       // claw = hardwareMap.get(Servo.class,"claw");
        rotateClaw = hardwareMap.get(CRServo.class,"rotateClaw");

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
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Detected Stones", pipeline.getDetectedStones().size());

            if (gamepad1.triangle) {
                pipeline.targetColor = "Blue";
            } else if (gamepad1.cross) {
                pipeline.targetColor = "Red";
            } else if (gamepad1.square) {
                pipeline.targetColor = "Yellow";
            }

            // Get the target angle from the pipeline
            double targetAngle = pipeline.targetAngle;

            //NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
            double output = (((targetAngle - 0) * (1 - 0)) / (180 - 0)) + 0;



//
//
//
//            if(output > 0.55)
//                rotateClaw.setPower(-0.03);
//            else if(output > 0.1 && output < 0.9)
//                rotateClaw.setPower(0.03);
//            else
//                rotateClaw.setPower(0);

//
//            if(targetAngle>170 && targetAngle<10){
//                rotateClaw.setPower(0);
//            }
//            else if (gamepad1.right_stick_x>0.1){
//                rotateClaw.setPower(gamepad1.right_stick_x*0.05);
//            }
//            else if (gamepad1.left_stick_x>0.1){
//                rotateClaw.setPower(-gamepad1.left_stick_x*0.05);
//            }
//
            if(targetAngle < 160 && targetAngle > 90 && pipeline.getDetectedStones().size()>0) {
                rotateClaw.setPower(-0.1);
            }
            else if(targetAngle > 20 && targetAngle <= 90 && pipeline.getDetectedStones().size()>0){
                rotateClaw.setPower(0.1);
            }
            else if(targetAngle > 20 && targetAngle <= 90 && pipeline.getDetectedStones().size()>0){
                rotateClaw.setPower(0.1);
            }
            else if(targetAngle > 20 && targetAngle <= 90 && pipeline.getDetectedStones().size()>0){
                rotateClaw.setPower(0.1);
            }
            else if(pipeline.getDetectedStones().size()==0){
                rotateClaw.setPower(0);
            }
            else {
                rotateClaw.setPower(0);
            }

//            if(targetAngle > 10 && targetAngle < 170)
//                rotateClaw.setPower(0.05);
//            else
//                rotateClaw.setPower(0);






//            if(output > 0.5 && output<0.97){//
//            rotateClaw.setPower(-0.05);
//            } else if (output <= 0.5 && output>0.03) {
//                rotateClaw.setPower(0.05);
//            } else{
//                rotateClaw.setPower(0);
//            }

            telemetry.addData("Target angle", targetAngle);
            telemetry.addData("Target angle", targetAngle);
            telemetry.addData("Ranged Target Angle", output);

          //  telemetry.addData("Claw Position", claw.getPosition());
            telemetry.update();

            sleep(100);
        }

        // Stop the camera when the OpMode ends
        camera.stopStreaming();
    }
}
