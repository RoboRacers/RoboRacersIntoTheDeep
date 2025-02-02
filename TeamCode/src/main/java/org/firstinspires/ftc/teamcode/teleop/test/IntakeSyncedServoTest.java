package org.firstinspires.ftc.teamcode.teleop.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

//@Disabled // Comment out this line to add to the opmode list

@Config
@TeleOp(name = "Intake Synced Servo Test", group = "Test")
public class IntakeSyncedServoTest extends LinearOpMode {

    DcMotorImplEx intakeMotor;
    ServoImplEx flipRight;
    ServoImplEx flipLeft;
    ServoImplEx v4bServo;

    public static boolean rightEnable = false;
    public static double target = 0;
    public static boolean leftEnable = true;
    public static double v4bTarget = 0;
    public static boolean v4bEnable = false;



    @Override
    public void runOpMode() throws InterruptedException {

        flipRight = hardwareMap.get(ServoImplEx.class, "intakeFlipRight");
        flipLeft = hardwareMap.get(ServoImplEx.class, "intakeFlipLeft");
        v4bServo = hardwareMap.get(ServoImplEx.class, "intakeV4b");
intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
        while (opModeInInit()) {
        }


        while (!isStopRequested()) {



            if (gamepad1.a){
                target = 0;
            }

            else if(gamepad1.x){
                target = 0.5;
            }
            else if (gamepad1.y){
                target = 1;
            } else if (gamepad1.left_stick_y >0.1 && gamepad1.b) {
                target = gamepad1.left_stick_y;
            }

            if(gamepad1.dpad_up){
                v4bTarget = 1;
            }
            else if(gamepad1.dpad_right){
                v4bTarget = 0.5;
            }
            else if(gamepad1.dpad_down){
                v4bTarget = 0;
            }
            else if (gamepad1.right_stick_y >0.1 && gamepad1.dpad_right) {
                v4bTarget = gamepad1.right_stick_y;
            }


            flipRight.setPosition(target);
            flipLeft.setPosition(target);
            v4bServo.setPosition(v4bTarget);



            intakeMotor.setPower(gamepad1.left_stick_y);

            telemetry.addData("target", target);
            telemetry.addData("v4b target", v4bTarget);
            telemetry.addData("flipRight", flipRight.getPosition());
            telemetry.addData("flipLeft", flipLeft.getPosition());
            telemetry.addData("v4b Servo", v4bServo.getPosition());
            telemetry.update();
        }
    }
}
