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

    Deposit deposit;



    @Override
    public void runOpMode() throws InterruptedException {

        deposit =  new Deposit(hardwareMap);

        Gamepad prevGamepad1;

        while (opModeInInit()) {
        }



        while (!isStopRequested()) {



            telemetry.update();
        }
    }
}
