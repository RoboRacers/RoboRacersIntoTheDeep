package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    public static double kP = 0.1;
    public static  double kI = 0;
    public static  double kD = 0.05;
    public static double target = 100;
    PIDController pitchControl;
    final double ticksToDegrees = (double) 90 /380;

    //Slides Stuff
    public DcMotorImplEx slidesMotor;

    //flip stuff
    Servo uno; //left flip
    Servo dos; //right flip


    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit()) {
            slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
            pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
            pitchControl = new PIDController(kP, kI, kD);

            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            uno = hardwareMap.get(Servo.class, "flipLeft");
            dos = hardwareMap.get(Servo.class, "flipRight");
        }

        while (!isStopRequested()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();


            pitchControl.setCoeffiecents(kP, kI, kD);
            if (gamepad2.triangle) {
                target = 300;
            } else if (gamepad2.cross) {
                target = 150;
            } else if (gamepad2.circle) {
                target = 200;
            }
            pitchControl.setSetpoint(target);
            double feedforward = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - 20) * ticksToDegrees)) + 0;
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
            pitchMotor.setPower(feedforward + pid);

            slidesMotor.setPower(gamepad2.right_stick_x * 0.7);

            //ALWAYS MULTIPLY THE RIGHT FLIP OR DOS BY 0.95 TO MAKE IT SYNC WITH THE LEFT DEPOSIT OR UNO
            if(gamepad1.dpad_down){
                //need to test values
                uno.setPosition(0.75);
                dos.setPosition(0.75*0.95);
            }
            else if(gamepad1.dpad_up){
                uno.setPosition(0.2);
                dos.setPosition(0.2*0.95);
            }





            telemetry.addData("Slides Power", slidesMotor.getPower());
            telemetry.addData("slides Pos", slidesMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Position", pitchMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", pitchMotor.getPower());
            telemetry.addData("Pitch Motor Angle", (pitchMotor.getCurrentPosition() - 20 ) * ticksToDegrees);
            telemetry.update();

        }
    }
}
