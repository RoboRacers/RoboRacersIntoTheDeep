package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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
        Rolling rollingIntake = new Rolling();
        Deposit deposit = new Deposit();

        while (opModeInInit()) {

        }

        while (!isStopRequested()) {
            // Gamepad 1 Controls
            robot.drive.setWeightedDrivePower(new Pose2d(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x));

            if (gamepad1.dpad_up){
                // extend slides
                rollingIntake.setSlidePower(0.5);
            }
            else if (gamepad1.dpad_down) {
                // retract slides
                rollingIntake.setSlidePower(-0.5);
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
                deposit.setSlidePos(1); //change value
            }
            else if (gamepad2.cross) {
                // lower slides
                deposit.setSlidePos(0);//change value
            }

            // End Gamepad 2 Controls
            robot.update();

        }
    }
}
