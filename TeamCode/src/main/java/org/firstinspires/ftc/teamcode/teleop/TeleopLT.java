package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Assembly;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Teleop LT", group = "0000-Final")
public class TeleopLT extends LinearOpMode {

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
                 intakeFlip = 0.2;

                } else if (gamepad1.dpad_up) { //neutral
                   intake4b = 0.55;
                   intakeFlip = 0.5;

                }

                if (gamepad1.right_trigger > 0.1) {intakeMotor.setPower(0.8);}
                else if (gamepad1.left_trigger > 0.1) {intakeMotor.setPower(-0.8);}
                else { intakeMotor.setPower(0);}

                if (gamepad1.dpad_right) {
                    slidesMotor.setPower(0.85);
                } else if (gamepad1.dpad_left) {
                    slidesMotor.setPower(-0.85);
                } else {
                    slidesMotor.setPower(0);
                }

                if(gamepad1.a){//specimen grab
                    deposit4b = 0.06;
                    depositFlip = 0.65;
                    extendo = 0.48;
                }else if(gamepad1.y){//going to score
                    depositFlip = 0.05;
                    extendo = 0.95;
                    deposit4b = 0.5;
                }

                if(gamepad1.x){//score
                    depositFlip = 0.18;
                    deposit4b = 0.35;
                    clawPos=0.82;
                }

                if(gamepad1.right_bumper){
                    clawPos = 0.91;
                }else if(gamepad1.left_bumper){
                    clawPos = 0.6;
                }

                intakeV4b.setPosition(intake4b);
                intakeFlipLeft.setPosition(intakeFlip-0.01);

                intakeFlipRight.setPosition(intakeFlip);

                depositV4bServo.setPosition(deposit4b);

                depositFlipLeft.setPosition(depositFlip);
                depositFlipRight.setPosition(depositFlip);

                extendoLeft.setPosition(extendo);
                extendoRight.setPosition(extendo);
                claw.setPosition(clawPos);


                telemetry.addData("Intake Flip l", intakeFlipLeft.getPosition());
                telemetry.addData("Intake Flip r", intakeFlipRight.getPosition());
                telemetry.addData("Deposit Flip l", depositFlipLeft.getPosition());
                telemetry.addData("Deposit Flip r", depositFlipRight.getPosition());
                telemetry.addData("Deposit v4b", depositV4bServo.getPosition());
                telemetry.addData("Intake v4b", intakeV4b.getPosition());
                telemetry.addData("Claw", claw.getPosition());
                telemetry.addData("Extendo l", extendoLeft.getPosition());
                telemetry.addData("Extendo r", extendoRight.getPosition());
                telemetry.update();


        }
    }
}
