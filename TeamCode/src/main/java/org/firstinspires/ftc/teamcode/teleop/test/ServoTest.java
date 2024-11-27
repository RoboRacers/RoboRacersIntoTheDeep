package org.firstinspires.ftc.teamcode.teleop.test;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends LinearOpMode {
    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "flipLeft");

        while (opModeInInit()) {

        }
        while (!isStopRequested()) {
            servo.setPosition(gamepad1.right_stick_x);
        }
    }
}
