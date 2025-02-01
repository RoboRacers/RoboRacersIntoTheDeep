package org.firstinspires.ftc.teamcode.teleop.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.Deposit;

//@Disabled // Comment out this line to add to the opmode list

@Config
@TeleOp(name = "Deposit Test", group = "Test")
public class DepositTest extends LinearOpMode {
    ServoImplEx clawPitch;
    public static double clawPitchPosition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        clawPitch = hardwareMap.get(ServoImplEx.class, "depositV4bServo");



        while (opModeInInit()) {
        }


        while (!isStopRequested()) {
            clawPitch.setPosition(clawPitchPosition);


            telemetry.update();
        }
    }
}
