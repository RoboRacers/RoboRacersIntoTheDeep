package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class IntakeTesty extends LinearOpMode {
    DcMotor slideMotor;
    ServoImplEx flipLeft;
    ServoImplEx flipRight;
    CRServoImplEx intakeMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        slideMotor = hardwareMap.get(DcMotor.class, "Horizontal_Slides");
        flipLeft = hardwareMap.get(ServoImplEx.class, "Flip_Left_Intake");
        flipRight = hardwareMap.get(ServoImplEx.class, "Flip_Right_Intake");
        intakeMotor = hardwareMap.get(CRServoImplEx.class, "Servo_Intake");

        waitForStart();

        while (!isStopRequested()){
            if(gamepad1.right_trigger > 0.1){
                intakeMotor.setPower(1);
            }else if (gamepad1.left_trigger > 0.1){
                intakeMotor.setPower(-1);
            }else{
                intakeMotor.setPower(0);
            }

            if(gamepad1.dpad_down){
                flipRight.setPosition(0.8);
                flipLeft.setPosition(0.8);
            }else if(gamepad1.dpad_up){
                flipLeft.setPosition(0.2);
                flipRight.setPosition(0.2);
            }

            if(gamepad1.left_bumper){
                slideMotor.setPower(1);
            }else if(gamepad1.right_bumper){
                slideMotor.setPower(-1);
            }


            telemetry.addData("FlipRight Position", flipRight.getPosition());
            telemetry.addData("FFlipLeft Position", flipLeft.getPosition());
            telemetry.update();
        }
    }
}