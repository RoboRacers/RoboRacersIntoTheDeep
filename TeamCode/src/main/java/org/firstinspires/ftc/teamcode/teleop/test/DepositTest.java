package org.firstinspires.ftc.teamcode.teleop.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

//@Disabled // Comment out this line to add to the opmode list

@Config
@TeleOp(name = "AAAAAAA", group = "Test")
public class DepositTest extends LinearOpMode {

    public ServoImplEx flipRight;
    public ServoImplEx flipLeft;
    public ServoImplEx v4bServo;
    public ServoImplEx claw;
    public ServoImplEx extendoRight;
    public ServoImplEx extendoLeft;

    public static double target = 0;
    public static boolean rightEnable = false;
    public static boolean leftEnable = true;
    public static double v4bTarget = 0;
    public static double clawTarget = 0;
    public static double offset = 0.01;
    public static boolean v4bEnable = false;
    public static double extendoTarget = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        //intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");

        flipRight = hardwareMap.get(ServoImplEx.class, "depositFlipRight");
        flipLeft = hardwareMap.get(ServoImplEx.class, "depositFlipLeft");
        v4bServo = hardwareMap.get(ServoImplEx.class, "depositV4bServo");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        extendoRight = hardwareMap.get(ServoImplEx.class, "extendoRight");
        extendoLeft = hardwareMap.get(ServoImplEx.class, "extendoLeft");
        extendoRight.setDirection(Servo.Direction.REVERSE);
        flipLeft.setDirection(Servo.Direction.REVERSE);


        flipLeft.setDirection(Servo.Direction.REVERSE);
        while (opModeInInit()) {
        }


        while (!isStopRequested()) {

            if ( gamepad1.dpad_right) {
                flipLeft.setPosition(gamepad1.left_stick_y);
                flipRight.setPosition(gamepad1.left_stick_y);
                telemetry.addData("pos", gamepad1.left_stick_y);
                telemetry.update();

            }
            else if ( gamepad1.dpad_up) {
                flipLeft.setPosition(0.75);
                flipRight.setPosition(0.75);
//                telemetry.addData("claw", extendoRight.getPosition());
//                telemetry.update();

            }else if ( gamepad1.dpad_down) {
                flipLeft.setPosition(0.65);
                flipRight.setPosition(0.65);
//                telemetry.addData("claw", extendoRight.getPosition());
//                telemetry.update();

            }
            else if ( gamepad1.dpad_left) {
                flipLeft.setPosition(0.5);
                flipRight.setPosition(0.5);
//                telemetry.addData("claw", extendoRight.getPosition());
//                telemetry.update();

            }




            telemetry.update();
        }
    }
}
