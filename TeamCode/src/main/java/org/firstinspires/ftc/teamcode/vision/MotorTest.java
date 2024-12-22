package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Motor Test", group = "Vision")
public class MotorTest extends LinearOpMode {
    Servo motor;

    @Override
    public void runOpMode() throws InterruptedException {

       // claw = hardwareMap.get(Servo.class,"claw"); //0.17= min 0.96 = max
        motor = hardwareMap.get(Servo.class, "rotateClaw");

        // Open the camera


        waitForStart();

        while (opModeIsActive()) {


            if(gamepad1.dpad_left)
                motor.setPosition(gamepad1.right_stick_y);


            telemetry.addData("Motor Pos", motor.getPosition());
            telemetry.addData("Stick Pos", gamepad1.right_stick_y);

            telemetry.update();


        }

        // Stop the camera when the OpMode ends

    }
}
