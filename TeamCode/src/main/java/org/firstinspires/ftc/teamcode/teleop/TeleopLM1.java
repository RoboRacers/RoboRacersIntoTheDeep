package org.firstinspires.ftc.teamcode.teleop;



import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.DepositTesty;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Deposit;
import org.firstinspires.ftc.teamcode.teleop.Rolling;



@TeleOp(name = "LM1 Teleop", group = "16481-IntoTheDeep")
public class TeleopLM1 extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    Deposit deposit;
    Rolling rollingIntake;

    @Override
    public void runOpMode() throws InterruptedException {


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        deposit = new Deposit(hardwareMap);
        rollingIntake = new Rolling(hardwareMap);


//        Gamepad gamepad1 = new Gamepad();
//        Gamepad gamepad2 = new Gamepad();

        //Deposit deposit = new Deposit();
        /*
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        */

        while (opModeInInit()) {

        }

        while (!isStopRequested()) {
            // Gamepad 1 Controls

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            if (gamepad1.dpad_up){
                // extend slides
                rollingIntake.setSlidePos(10);
            }
            else if (gamepad1.dpad_down) {
                // retract slides
                rollingIntake.setSlidePos(2);
            }

            // Intake
            if (gamepad1.right_trigger > 0.2){
                rollingIntake.setIntake();
            }
            // Outake
            else if (gamepad1.left_trigger > 0.2){
                rollingIntake.setOutake();
            }
            else{
                rollingIntake.stopIntake();
            }

            if (gamepad1.triangle){
                rollingIntake.setIntakeUp();
            } else if (gamepad1.cross) {
                rollingIntake.setIntakeDown();
            }
            // End Gamepad 1 Controls

            // Gamepad 2 Controls
            // deposit claw
            if (gamepad2.left_trigger > 0.1){
                // claw close
                deposit.openClaw();
            }
            else if (gamepad2.right_trigger > 0.1){
                // claw open
                deposit.closeClaw();
            }
            // deposit flip
            if (gamepad2.dpad_down){
                // flip deposit thingy upwards
                deposit.goToGrab();
            }
            else if (gamepad2.dpad_up){
                // flip deposit thingy downwards
                deposit.goToRelease();
            }

            // vertical slides
            if (gamepad2.triangle){
                // raise slides
                deposit.setSlidePos(30); //change value
            }
            else if (gamepad2.cross) {
                // lower slides
                deposit.setSlidePos(0);//change value
            }

            // End Gamepad 2 Controls
//            rollingIntake.update();
//            deposit.update();
//            telemetry.update();

        }
    }
}
