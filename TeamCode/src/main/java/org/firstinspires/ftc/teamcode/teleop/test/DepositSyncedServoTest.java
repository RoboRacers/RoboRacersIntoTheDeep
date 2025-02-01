package org.firstinspires.ftc.teamcode.teleop.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

//@Disabled // Comment out this line to add to the opmode list

@Config
@TeleOp(name = "Deposit Synced Servo Test", group = "Test")
public class DepositSyncedServoTest extends LinearOpMode {

    ServoImplEx flipRight;
    ServoImplEx flipLeft;
    ServoImplEx v4bServo;
    ServoImplEx clawServo;

    public static double target = 0;
    public static boolean rightEnable = false;
    public static boolean leftEnable = true;
    public static double v4bTarget = 0;
    public static double clawTarget = 0;
    public static boolean v4bEnable = false;



    @Override
    public void runOpMode() throws InterruptedException {
        //intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");

        flipRight = hardwareMap.get(ServoImplEx.class, "depositFlipRight");
        flipLeft = hardwareMap.get(ServoImplEx.class, "depositFlipLeft");
        v4bServo = hardwareMap.get(ServoImplEx.class, "depositV4bServo");
        clawServo = hardwareMap.get(ServoImplEx.class, "claw");

        flipLeft.setDirection(Servo.Direction.REVERSE);
        while (opModeInInit()) {
        }


        while (!isStopRequested()) {

            flipRight.setPosition(target);
            flipLeft.setPosition(target);
            v4bServo.setPosition(v4bTarget);
            clawServo.setPosition(clawTarget);



            telemetry.addData("target", target);
            telemetry.addData("v4b target", v4bServo);
            telemetry.update();
        }
    }
}
