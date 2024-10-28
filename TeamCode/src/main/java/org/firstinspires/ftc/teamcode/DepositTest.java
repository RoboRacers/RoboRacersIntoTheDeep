package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;    
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name ="DepositWORK", group = "16481-IntoTheDeep")
public class DepositTest extends LinearOpMode {
    public DcMotorImplEx intakeMotorLeft;
    public DcMotorImplEx intakeMotorRight;

    public DcMotor slideMotor;
    public ServoImplEx flipLeftIntake;
    public ServoImplEx flipRightIntake;

    public CRServoImplEx rolling;



    public CRServoImplEx intakeMotor;
    public Servo depositLeft;
    public Servo depositRight;

    public ServoImplEx flipRightDeposit;
    public ServoImplEx flipLeftDeposit;
    public ServoImplEx pitch;
    public ServoImplEx claw;

    public DcMotorImplEx slidesRight;
    public DcMotorImplEx slidesLeft;


    @Override
    public void runOpMode() throws InterruptedException {


           // MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        slideMotor = hardwareMap.get(DcMotor.class, "Horizontal_Slides");
        flipLeftIntake = hardwareMap.get(ServoImplEx.class, "Flip_Left_Intake");
        flipRightIntake = hardwareMap.get(ServoImplEx.class, "Flip_Right_Intake");
        intakeMotor = hardwareMap.get(CRServoImplEx.class, "Servo_Intake");


//            depositRight = hardwareMap.get(ServoImplEx.class, "")


        flipRightDeposit = hardwareMap.get(ServoImplEx.class, "Flip_Right_Deposit");
        flipLeftDeposit = hardwareMap.get(ServoImplEx.class, "Flip_Left_Deposit");
        pitch = hardwareMap.get(ServoImplEx.class, "Pitch");
        claw = hardwareMap.get(ServoImplEx.class, "Claw");

        slidesRight = hardwareMap.get(DcMotorImplEx.class, "Slides_Right");
        slidesLeft = hardwareMap.get(DcMotorImplEx.class, "Slides_Left");

        slidesLeft.setDirection(DcMotorImplEx.Direction.REVERSE);
        intakeMotor.setDirection(CRServoImplEx.Direction.REVERSE);


            waitForStart();

            while (opModeIsActive()) {

                if (gamepad1.right_bumper){
                    // extend slides
                     slideMotor.setPower(0.4);
                }
                else if (gamepad1.left_bumper) {
                    // retract slides
                    slideMotor.setPower(-0.4);
                }
                else if (gamepad1.dpad_down) {
                    intakeMotor.setPower(-0.4);
                } else if (gamepad1.dpad_up) {
                    intakeMotor.setPower(0.7);
                } else {
                    slideMotor.setPower(0);
                    intakeMotor.setPower(0);
                }
                if (gamepad1.cross){
                    flipLeftIntake.setPosition(0.92);
                    flipRightIntake.setPosition(0.92);
                } else if (gamepad1.square) {
                    flipLeftIntake.setPosition(0.3);
                    flipRightIntake.setPosition(0.3);
                }
                else if (gamepad2.cross){
                    pitch.setPosition(0.275);
                }
                else if (gamepad2.square){
                    pitch.setPosition(0.28);
                }
                else if (gamepad2.circle){
                    pitch.setPosition(0);
                } else if (gamepad2.triangle){
                    pitch.setPosition(1);
                }
                else if(gamepad2.right_bumper){
                    claw.setPosition(0.45);
                }
                else if(gamepad2.left_bumper){
                    claw.setPosition(0.7);
                } else if (gamepad2.left_stick_x>0.1) {
                    flipLeftDeposit.setPosition(gamepad2.left_stick_x);
                    flipRightDeposit.setPosition(gamepad2.left_stick_x);
                }

                // Intake
//                if (gamepad1.right_trigger > 0.5){
//                    intakeMotor.setDirection(CRServoImpl.Direction.FORWARD);
//                    intakeMotor.setPower(0.5);
//                } else if (gamepad1.left_trigger > 0.5){
//                    intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//                    intakeMotor.setPower(0.5);
//
//                }
//
//                if (gamepad1.triangle){
//                     setIntakeUp();
//                } else if (gamepad1.cross) {
//                     setIntakeDown();
//                }




                //drive.updatePoseEstimate();
//                telemetry.addData("Flip Right value", flipRightIntake.getPosition());
//                telemetry.addData("Flip Left value", flipLeftIntake.getPosition());
//                telemetry.addData("Flip Left value", rolling.getPower());
//                telemetry.addData("ServoPos", depositRight.getPosition());
//                telemetry.addData("ServoPos", depositLeft.getPosition());
//
//                telemetry.addData("slidePosLeft", intakeMotorLeft.getCurrentPosition());
//                telemetry.addData("slidePosRight", intakeMotorRight.getCurrentPosition());
                telemetry.addData("Intake Slides", slideMotor.getCurrentPosition());

                telemetry.addData("Flip Left Deposit", flipLeftDeposit.getPosition());
                telemetry.addData("Flip Right Deposit", flipRightDeposit.getPosition());

                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
              //  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            }

    }
}
