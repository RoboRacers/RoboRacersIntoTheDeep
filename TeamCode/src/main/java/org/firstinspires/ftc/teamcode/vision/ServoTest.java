package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servo Control with Gamepad", group = "Examples")
public class ServoTest extends LinearOpMode {
    // Define the servo
    private Servo left;
    private Servo right;
    private Servo claw;
    private Servo rotate;


    @Override
    public void runOpMode() {
        // Initialize the servo from the hardware map
        left = hardwareMap.get(Servo.class, "flipLeft");
        right = hardwareMap.get(Servo.class, "flipRight");
        claw = hardwareMap.get(Servo.class, "claw");
        rotate = hardwareMap.get(Servo.class, "rotateClaw");


        // Set the servo to the initial position
        // Wait for the game to start
        telemetry.addData("Status", "Initialized. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Increase servo position with gamepad 'A'

            if(gamepad1.dpad_down){
                left.setPosition(gamepad1.right_stick_y);
                right.setPosition(gamepad1.right_stick_y*0.95);
            }
            if(gamepad1.dpad_left){
                claw.setPosition(gamepad1.right_stick_y);
            }if(gamepad1.dpad_right){
                rotate.setPosition(gamepad1.right_stick_y);
            }


        }
    }
}
