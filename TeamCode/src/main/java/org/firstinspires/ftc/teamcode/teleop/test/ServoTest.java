package org.firstinspires.ftc.teamcode.teleop.test;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends LinearOpMode {
    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "rotateClaw");

        while (opModeInInit()) {

        }


        while (!isStopRequested()) {
            servo.setPosition(gamepad1.left_stick_y);
            telemetry.addData("pos", servo.getPosition());
            telemetry.update();
        }
    }
}
