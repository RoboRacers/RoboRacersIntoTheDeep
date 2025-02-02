package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp(name = "Orion", group = "0000-Final")
public class Orion extends LinearOpMode {

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    DcMotorImplEx intakeMotor;
    DcMotorImplEx slidesMotor;
    ServoImplEx extendoLeft;
    ServoImplEx extendoRight;
    ServoImplEx depositFlipLeft;
    ServoImplEx depositFlipRight;
    ServoImplEx depositV4bServo;
    ServoImplEx claw;

    ServoImplEx intakeFlipLeft;
    ServoImplEx intakeFlipRight;
    ServoImplEx intakeV4b;

    double intakeFlip;

    double depositFlip;
    double extendo;

    double clawPos;
    double intake4b;
    double deposit4b;

//    ElapsedTime time = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");

        extendoLeft = hardwareMap.get(ServoImplEx.class, "extendoLeft");
        extendoRight = hardwareMap.get(ServoImplEx.class, "extendoRight");
        depositFlipLeft = hardwareMap.get(ServoImplEx.class, "depositFlipLeft");

        depositFlipRight = hardwareMap.get(ServoImplEx.class, "depositFlipRight");
        depositV4bServo = hardwareMap.get(ServoImplEx.class, "depositV4bServo");
        claw = hardwareMap.get(ServoImplEx.class, "claw");

        intakeFlipLeft = hardwareMap.get(ServoImplEx.class, "intakeFlipLeft");
        intakeFlipRight = hardwareMap.get(ServoImplEx.class, "intakeFlipRight");
        intakeV4b = hardwareMap.get(ServoImplEx.class, "intakeV4b");
        slidesMotor.setDirection(DcMotorImplEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorImplEx.Direction.REVERSE);
        extendoRight.setDirection(Servo.Direction.REVERSE);
        depositFlipLeft.setDirection(Servo.Direction.REVERSE);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        while (opModeInInit()) {
        }

        waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                TelemetryPacket packet = new TelemetryPacket();

                if (gamepad1.dpad_down) { //intake
                    intake4b = 0.4;
                    intakeFlip = 0.22;
                    intakeFlipLeft.setPosition(intakeFlip-0.01);
                    intakeV4b.setPosition(intake4b);
                    intakeFlipRight.setPosition(intakeFlip);

                } else if (gamepad1.dpad_up) { //neutral
                    intakeMotor.setPower(0);
                    intake4b = 0.3;
                    intakeFlip = 0.5;
                    intakeFlipLeft.setPosition(intakeFlip-0.01);
                    intakeV4b.setPosition(intake4b);
                    intakeFlipRight.setPosition(intakeFlip);

                }

                if (gamepad1.right_trigger > 0.1) {intakeMotor.setPower(0.8);}
                else if (gamepad1.left_trigger > 0.1) {intakeMotor.setPower(-0.8);}
                else{
                    intakeMotor.setPower(0);
                }

                if (gamepad1.dpad_right) {
                    slidesMotor.setPower(1);
                } else if (gamepad1.dpad_left) {
                    slidesMotor.setPower(-1);
                } else {
                    slidesMotor.setPower(0);
                }

            if(gamepad1.a){//specimen grab
               depositV4bServo.setPosition(0.03);
                depositFlipLeft.setPosition(0.6);
                depositFlipRight.setPosition(0.6);
                extendoLeft.setPosition(0.48);
                extendoRight.setPosition(0.48);




            }
            if(gamepad1.x){//going to score


                depositV4bServo.setPosition(0.5);

                depositFlipLeft.setPosition(0.05);
                depositFlipRight.setPosition(0.05);

                extendoLeft.setPosition(0.9);
                extendoRight.setPosition(0.9);

            }
            if(gamepad1.y){//going to score


                depositV4bServo.setPosition(0.35);

                depositFlipLeft.setPosition(0.14);
                depositFlipRight.setPosition(0.14);

                extendoLeft.setPosition(0.7);
                extendoRight.setPosition(0.7);
                claw.setPosition(0.82);

//
            }


                if(gamepad1.right_bumper){
                    clawPos = 0.95;
                    claw.setPosition(clawPos);
                }else if(gamepad1.left_bumper){
                    clawPos = 0.6;
                    claw.setPosition(clawPos);
                }



//                intakeV4b.setPosition(intake4b);
//                intakeFlipLeft.setPosition(intakeFlip-0.01);
//
//                intakeFlipRight.setPosition(intakeFlip);

//                depositV4bServo.setPosition(deposit4b);
//
//                depositFlipLeft.setPosition(depositFlip);
//                depositFlipRight.setPosition(depositFlip);
//
//                extendoLeft.setPosition(extendo);
//                extendoRight.setPosition(extendo);
//                claw.setPosition(clawPos);
                telemetry.update();


        }
    }
}
