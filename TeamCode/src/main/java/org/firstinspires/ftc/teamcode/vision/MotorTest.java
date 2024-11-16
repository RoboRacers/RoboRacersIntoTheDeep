package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Motor Test", group = "Vision")
public class MotorTest extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

       // claw = hardwareMap.get(Servo.class,"claw");
        motor = hardwareMap.get(DcMotor.class, "motor");

        // Open the camera


        waitForStart();

        while (opModeIsActive()) {


            motor.setPower(gamepad1.right_stick_x);


            telemetry.addData("Motor Pos", motor.getCurrentPosition());
            telemetry.addData("Motor Pos", motor.getPower());



          //  telemetry.addData("Claw Position", claw.getPosition());
            telemetry.update();


        }

        // Stop the camera when the OpMode ends

    }
}
