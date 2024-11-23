package org.firstinspires.ftc.teamcode.teleop;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@Disabled // Comment out this line to add to the opmode list
@TeleOp(name = "BLANK", group = "Test")
public class BlankTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        while (opModeInInit()) {
        }


        while (!isStopRequested()) {

        }
    }
}
