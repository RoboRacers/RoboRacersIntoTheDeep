package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.PIDController;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "Total Deposit Test", group = "Test")
public class TotalDepositTest extends LinearOpMode {
    //Pitch Stuff
    public DcMotorImplEx pitchMotor;
    public static double kG = 0.35;
    public static double kP = 0.05;
    public static  double kI = 0;
    public static  double kD = 0.0003;
    public static double target = 100;
    PIDController pitchControl;
    final double ticksToDegrees = (double) 90 /334;

    //Slides Stuff
    public DcMotorImplEx slidesMotor;

    //flip stuff
    Servo uno; //left flip
    Servo dos; //right flip
    double flipPos;
    Servo claw;
    //CRServo rotate;


    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
        pitchControl = new PIDController(kP, kI, kD);
        pitchControl.setErrorTolerance(18);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        uno = hardwareMap.get(Servo.class, "flipLeft");
        dos = hardwareMap.get(Servo.class, "flipRight");

        claw = hardwareMap.get(Servo.class, "claw");
        // rotate = hardwareMap.get(CRServo.class, "rotateClaw");

        while (opModeInInit()) {
            flipPos = 1;
            uno.setPosition(flipPos);
            dos.setPosition(flipPos * 0.95);


        }

        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        target = 200;
        while (!isStopRequested()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();


            pitchControl.setCoefficients(kP, kI, kD);

            if (gamepad2.triangle) {
                target = 900;
            } else if (gamepad2.cross) {
                target = 250;
            } else if (gamepad2.circle) {
                target = 500;
            }
            pitchControl.setSetpoint(target);
            double feedforward = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - 45) * ticksToDegrees)) + 0;
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
            pitchMotor.setPower(feedforward + pid);

            slidesMotor.setPower(gamepad2.right_stick_x * 0.7);

            //ALWAYS MULTIPLY THE RIGHT FLIP OR DOS BY 0.95 TO MAKE IT SYNC WITH THE LEFT DEPOSIT OR UNO
            if(gamepad2.dpad_up){
                flipPos = 0.75;
            }else if(gamepad2.dpad_down){
                flipPos = 0.25;
            }else if(gamepad2.dpad_right){
                flipPos = 0.5;
            } else if (gamepad2.dpad_left) {
                flipPos = 1;
            }

            uno.setPosition(flipPos);
            dos.setPosition(flipPos * 0.95);

            if (gamepad2.right_bumper){
                claw.setPosition(1);
            }else if(gamepad2.left_bumper){
                claw.setPosition(0);
            }




            telemetry.addData("Slides Power", slidesMotor.getPower());
            telemetry.addData("slides Pos", slidesMotor.getCurrentPosition());
            telemetry.addData("Target value Pitch", target);
            telemetry.addData("Feedforward", feedforward);
            telemetry.addData("PID Values", pid);
            telemetry.addData("Pitch Motor Position", pitchMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", pitchMotor.getPower());
            telemetry.addData("Pitch Motor Angle", (pitchMotor.getCurrentPosition() - 20 ) * ticksToDegrees);
            telemetry.addData("Uno pos", uno.getPosition());
            telemetry.addData("Dos pos", dos.getPosition());
            telemetry.update();

        }
    }
}
