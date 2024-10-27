package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.DepositTesty;
import org.firstinspires.ftc.teamcode.teleop.Deposit;
import org.firstinspires.ftc.teamcode.teleop.Rolling;



@TeleOp(name = "LM1 Teleop", group = "16481-IntoTheDeep")
public class TeleopLM1 extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    public Deposit deposit = new Deposit();
    public Rolling rollingIntake = new Rolling();

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("Fl");
        backLeftMotor = hardwareMap.dcMotor.get("Bl");
        frontRightMotor = hardwareMap.dcMotor.get("Fr");
        backRightMotor = hardwareMap.dcMotor.get("Br");


//        Gamepad gamepad1 = new Gamepad();
//        Gamepad gamepad2 = new Gamepad();

        //Deposit deposit = new Deposit();

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeInInit()) {

        }

        while (!isStopRequested()) {
            // Gamepad 1 Controls
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            if (gamepad1.dpad_up){
                // extend slides
                rollingIntake.setSlidePos(100);
            }
            else if (gamepad1.dpad_down) {
                // retract slides
                rollingIntake.setSlidePos(0);
            }
            else{
                rollingIntake.setSlidePower(0);
            }
            // Intake
            if (gamepad1.right_trigger > 0.1){
                rollingIntake.setIntake();
            }
            // Outake
            else if (gamepad1.left_trigger > 0.1){
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
            if (gamepad2.left_bumper){
                // claw close
                deposit.closeClaw();
            }
            else if (gamepad2.right_bumper){
                // claw open
                deposit.openClaw();
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
                deposit.setSlidePos(50); //change value
            }
            else if (gamepad2.cross) {
                // lower slides
                deposit.setSlidePos(0);//change value
            }

            // End Gamepad 2 Controls
            rollingIntake.update();
            deposit.update();
            telemetry.update();

        }
    }
}
