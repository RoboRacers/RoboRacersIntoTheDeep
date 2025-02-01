package org.firstinspires.ftc.teamcode.teleop.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

//@Disabled // Comment out this line to add to the opmode list

@Config
@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends LinearOpMode {

    ServoImplEx c0;
    ServoImplEx c1;
    ServoImplEx c2;
    ServoImplEx c3;
    ServoImplEx c4;
    ServoImplEx c5;
    ServoImplEx e0;
    ServoImplEx e1;
    ServoImplEx e2;
    ServoImplEx e3;
    ServoImplEx e4;
    ServoImplEx e5;

    public static int currentServo = 0;
    public static boolean controlHub = true;

    @Override
    public void runOpMode() throws InterruptedException {
        //intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");

        c0 = hardwareMap.get(ServoImplEx.class, "c0");
        c1 = hardwareMap.get(ServoImplEx.class, "c1");
        c2 = hardwareMap.get(ServoImplEx.class, "c2");
        c3 = hardwareMap.get(ServoImplEx.class, "c3");
        c4 = hardwareMap.get(ServoImplEx.class, "c4");
        c5 = hardwareMap.get(ServoImplEx.class, "c5");
        e0 = hardwareMap.get(ServoImplEx.class, "e0"); // extendo left
        e1 = hardwareMap.get(ServoImplEx.class, "e1"); // left deposit flip
        e2 = hardwareMap.get(ServoImplEx.class, "e2"); // deposit cv4b
        e3 = hardwareMap.get(ServoImplEx.class, "e3");  // right deposit flip
        e4 = hardwareMap.get(ServoImplEx.class, "e4"); // claw
        e5 = hardwareMap.get(ServoImplEx.class, "e5"); // extendo right


        c0.setPwmDisable();
        c1.setPwmDisable();
        c2.setPwmDisable();
        c3.setPwmDisable();
        c4.setPwmDisable();
        c5.setPwmDisable();
        e0.setPwmDisable();
        e1.setPwmDisable();
        e2.setPwmDisable();
        e3.setPwmDisable();
        e4.setPwmDisable();
        e5.setPwmDisable();

        while (opModeInInit()) {
        }


        while (!isStopRequested()) {

            switch (currentServo) {
                case 0:
                    if (controlHub) {
                        c0.setPwmEnable();
                        c0.setPosition(0.2);
                    } else {
                        e0.setPwmEnable();
                        e0.setPosition(0.2);
                    }
                    break;
                case 1:
                    if (controlHub) {
                        c1.setPwmEnable();
                        c1.setPosition(0.2);
                    } else {
                        e1.setPwmEnable();
                        e1.setPosition(0.2);
                    }
                    break;
                case 2:
                    if (controlHub) {
                        c2.setPwmEnable();
                        c2.setPosition(0.2);
                    } else {
                        e2.setPwmEnable();
                        e2.setPosition(0.2);
                    }
                    break;
                case 3:
                    if (controlHub) {
                        c3.setPwmEnable();
                        c3.setPosition(0.2);
                    } else {
                        e3.setPwmEnable();
                        e3.setPosition(0.2);
                    }
                    break;
                case 4:
                    if (controlHub) {
                        c4.setPwmEnable();
                        c4.setPosition(0.2);
                    } else {
                        e4.setPwmEnable();
                        e4.setPosition(0.2);
                    }
                    break;
                case 5:
                    if (controlHub) {
                        c5.setPwmEnable();
                        c5.setPosition(0.2);
                    } else {
                        e5.setPwmEnable();
                        e5.setPosition(0.2);
                    }
                    break;
            }

            if (gamepad1.a) {
                c0.setPwmDisable();
                c1.setPwmDisable();
                c2.setPwmDisable();
                c3.setPwmDisable();
                c4.setPwmDisable();
                c5.setPwmDisable();
                e0.setPwmDisable();
                e1.setPwmDisable();
                e2.setPwmDisable();
                e3.setPwmDisable();
                e4.setPwmDisable();
                e5.setPwmDisable();
            }

            telemetry.addData("current servo", currentServo);
            telemetry.addData("control hub", controlHub);
            telemetry.update();
        }
    }
}
