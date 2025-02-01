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

        while (opModeInInit()) {
        }


        while (!isStopRequested()) {

            flipRight.setPosition(target);
            flipLeft.setPosition(target);
            v4bServo.setPosition(v4bTarget);

            if (rightEnable)
                flipRight.setPwmEnable();
            else
                flipRight.setPwmDisable();

            if (leftEnable)
                flipLeft.setPwmEnable();
            else
                flipLeft.setPwmDisable();

            if (v4bEnable)
                v4bServo.setPwmEnable();
            else
                v4bServo.setPwmDisable();

            intakeMotor.setPower(gamepad1.left_stick_y);

            telemetry.addData("target", target);
            telemetry.addData("v4b target", v4bServo);
            telemetry.update();
        }
    }
}
