package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Assembly;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {

    DcMotorImplEx pitchMotor;

    ElapsedTime runtime = new ElapsedTime();

    public List<Actions> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {

        runtime.reset();

        DcMotorImplEx pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");


        while (opModeInInit()) {

        }

        waitForStart();



        while (!isStopRequested()) {

            pitchMotor.setPower(gamepad1.left_stick_y);
        }
    }

}