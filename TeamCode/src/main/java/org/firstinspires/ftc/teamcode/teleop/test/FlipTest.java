package org.firstinspires.ftc.teamcode.teleop.test;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled // Comment out this line to add to the opmode list
@Disabled
@TeleOp(name = "FlipTest", group = "Test")
public class FlipTest extends LinearOpMode {
Servo uno;
Servo dos;
    @Override
    public void runOpMode() throws InterruptedException {
    uno = hardwareMap.get(Servo.class, "flipLeft");
    dos = hardwareMap.get(Servo.class, "flipRight");

        while (opModeInInit()) {
        }


        while (!isStopRequested()) {


//            if(gamepad1.cross){
//                dos.setPosition(0.952);
//            } else if (gamepad1.circle) {
//                dos.setPosition(0.5*0.95);
//                uno.setPosition(0.5);
//            }
//            else if (gamepad1.square) {
//                dos.setPosition(0.975);
//            }
//            else if (gamepad1.triangle) {
//                dos.setPosition(0.985);
//            }else{
//                dos.setPosition(gamepad1.right_stick_x*0.95);
//                uno.setPosition(gamepad1.left_stick_x);
//            }

//Above in motor Test

            //ALWAYS MULTIPLY THE RIGHT FLIP OR DOS BY 0.95 TO MAKE IT SYNC WITH THE LEFT DEPOSIT OR UNO
            if(gamepad1.cross){
                uno.setPosition(0.25);
                dos.setPosition(0.25*0.95);
            }
            else if(gamepad1.circle){
                uno.setPosition(0.5);
                dos.setPosition(0.5*0.95);
            }
            else if(gamepad1.triangle){
                uno.setPosition(0.75);
                dos.setPosition(0.75*0.95);
            }
            else{
                uno.setPosition(gamepad1.touchpad_finger_1_x);
                dos.setPosition(gamepad1.touchpad_finger_1_x*0.94);

            }
            telemetry.addData("Uno pos", uno.getPosition());
            telemetry.addData("Dos pos", dos.getPosition()/0.95);
            telemetry.update();


        }
    }
}
