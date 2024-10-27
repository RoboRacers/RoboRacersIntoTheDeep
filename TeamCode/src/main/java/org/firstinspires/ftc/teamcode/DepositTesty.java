package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.teleop.PIDController;

@TeleOp(name = "Deposit Test Here", group = "16481-IntoTheDeep")
public class DepositTesty extends LinearOpMode {
    ServoImplEx flipRight;
    ServoImplEx flipLeft;
    ServoImplEx pitch;
    ServoImplEx claw;

    DcMotorImplEx slidesRight;
    DcMotorImplEx slidesLeft;

    PIDController slidesPID = new PIDController(0.1, 0.01, 0.05);

    @Override
    public void runOpMode() throws InterruptedException {
        flipRight = flipRight = hardwareMap.get(ServoImplEx.class, "Flip_Right_Deposit");
        flipLeft = hardwareMap.get(ServoImplEx.class, "Flip_Left_Deposit");
        pitch = hardwareMap.get(ServoImplEx.class, "Pitch");
        claw = hardwareMap.get(ServoImplEx.class, "Claw");
        slidesRight = hardwareMap.get(DcMotorImplEx.class, "Slides_Right");
        slidesLeft = hardwareMap.get(DcMotorImplEx.class, "Slides_Left");


        while(!isStopRequested()){

            if(gamepad1.dpad_down){

                flipRight.setPosition(0.8);
                flipLeft.setPosition(0.8);
                pitch.setPosition(0.3);

            }else if(gamepad1.dpad_up){
                flipRight.setPosition(0);
                flipLeft.setPosition(0);
                pitch.setPosition(0.28);
            }



            if(gamepad1.right_bumper) {claw.setPosition(0.4);}

            if (gamepad1.left_bumper) {claw.setPosition(0.6);}

            telemetry.addData("FlipRight Position", flipRight.getPosition());
            telemetry.addData("FFlipLeft Position", flipLeft.getPosition());
            telemetry.addData("Pitch", pitch.getPosition());
            telemetry.addData("Claw", pitch.getPosition());

        }

    }
}