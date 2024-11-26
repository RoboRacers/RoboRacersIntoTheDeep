package org.firstinspires.ftc.teamcode.teleop;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled // Comment out this line to add to the opmode list
@TeleOp(name = "MotorTest", group = "Test")
public class MotorTest extends LinearOpMode {
Servo uno;
Servo dos;
    @Override
    public void runOpMode() throws InterruptedException {
    uno = hardwareMap.get(Servo.class, "flipLeft");
        dos = hardwareMap.get(Servo.class, "flipRight");

        while (opModeInInit()) {
        }


        while (!isStopRequested()) {

//ALWAYS MULTIPLY THE RIGHT FLIP OR DOS BY 0.95 TO MAKE IT SYNC WITH THE LEFT DEPOSIT OR UNO
            if(gamepad1.cross){
                dos.setPosition(0.952);
            } else if (gamepad1.circle) {
                dos.setPosition(0.5*0.95);
                uno.setPosition(0.5);
            }
            else if (gamepad1.square) {
                dos.setPosition(0.975);
            }
            else if (gamepad1.triangle) {
                dos.setPosition(0.985);
            }else{
                dos.setPosition(gamepad1.right_stick_x*0.95);
                uno.setPosition(gamepad1.left_stick_x);
            }
            telemetry.addData("Uno pos", uno.getPosition());
            telemetry.addData("Dos pos", dos.getPosition());
            telemetry.update();


        }
    }
}
