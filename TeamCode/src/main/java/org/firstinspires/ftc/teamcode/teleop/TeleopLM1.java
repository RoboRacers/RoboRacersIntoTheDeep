package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotCore;
import org.firstinspires.ftc.teamcode.robot.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.robot.subsystems.Rolling;


@TeleOp(name = "LM1 Teleop", group = "16481-IntoTheDeep")
public class TeleopLM1 extends LinearOpMode {

    RobotCore robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);
        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();
        Rolling rollingIntake = new Rolling(hardwareMap);
        Deposit deposit = new Deposit();

        while (opModeInInit()) {

        }

        while (!isStopRequested()) {
            // Gamepad 1 Controls
            robot.drive.setWeightedDrivePower(new Pose2d(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x));
            // Slides
            if (gamepad1.dpad_up){
                // extend slides
            }
            else if (gamepad1.dpad_down) {
                // retract slides
            }
            // Intake
            if (gamepad1.right_trigger > 0.1){
                rollingIntake.setIntakePower(gamepad1.right_trigger);
            }
            // Outake
            else if (gamepad1.left_trigger > 0.1){
                rollingIntake.setIntakePower(-gamepad1.left_trigger);
            }
            else{
                rollingIntake.setIntakePower(0);
            }
            // End Gamepad 1 Controls
            // Gamepad 2 Controls
            // deposit claw
            boolean depositClawClosed = false;
            if (gamepad2.cross && depositClawClosed == false){
                // claw close
                // depositClawServo.setPosition();     find this value later
                depositClawClosed = true;
            }
            else if (gamepad2.cross){
                // claw open
                // depositClawServo.setPosition();     find this value later
                depositClawClosed = false;
            }
            // deposit flip
            if (gamepad2.left_stick_y > 0){
                // flip deposit thingy upwards
            }
            else if (gamepad2.left_stick_y < 0){
                // flip deposit thingy downwards
            }
            // deposit claw angle
            if (gamepad2.right_stick_y > 0){
                // angle the deposit claw upwards/towards the back of the bot
            }
            else if (gamepad2.right_stick_y < 0){
                // angle the deposit claw downards/towards the front of the bot
            }
            // vertical slides
            if (gamepad2.dpad_up){
                // extend slides
            }
            else if (gamepad2.dpad_down){
                // retract slides
            }
            // End Gamepad 2 Controls
            robot.update();
        }
    }
}
