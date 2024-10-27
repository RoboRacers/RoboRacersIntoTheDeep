package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.useless.TankDrive;
@TeleOp(name ="Deposit", group = "16481-IntoTheDeep")
public class DepositTest extends LinearOpMode {
    public DcMotorImplEx intakeMotorLeft;
    public DcMotorImplEx intakeMotorRight;
    public ServoImplEx flipLeft;

    public ServoImplEx depositRight;

    public CRServoImplEx rolling;

    public ServoImplEx flipRight;
    public DcMotorEx intakeSlide;

    public Servo depositLeft;
    public Servo deposoitRight;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
           // MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            flipLeft = hardwareMap.get(ServoImplEx.class, "flipLeft");
            flipRight = hardwareMap.get(ServoImplEx.class, "flipRight");
            intakeMotorLeft = hardwareMap.get(DcMotorImplEx.class,"intakeRollLeft");
            intakeMotorRight = hardwareMap.get(DcMotorImplEx.class,"intakeRollRight");
//            depositRight = hardwareMap.get(ServoImplEx.class, "")
            rolling = hardwareMap.get(CRServoImplEx.class,"rolling");
            intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");

            depositLeft = hardwareMap.get(Servo.class, "depLeft");
            deposoitRight = hardwareMap.get(Servo.class, "depRight");

            waitForStart();

            while (opModeIsActive()) {
//                if (gamepad1.a){
//                    flipLeft.setPosition(0.7);
//                }
//                else if(gamepad1.b){
                //flipRight = 0.7182
//                    flipRight.setPosition(0.3);
//                }
//                else if(gamepad1.right_trigger>0.1){
//                    rolling.setPower(0.5);
//                }
                //flipLeft.setDirection(ServoImplEx.Direction.REVERSE);
                if (gamepad2.dpad_up){

                    flipLeft.setPosition(0.719);
                    flipRight.setPosition(0.7);
                } else if (gamepad2.dpad_down) {

                    flipLeft.setPosition(0.2);
                    flipRight.setPosition(0.2);
                }
                else if (gamepad2.dpad_left) {

                    flipLeft.setPosition(1);
                    flipRight.setPosition(1);
                }
                else if (gamepad2.dpad_right) {

                    flipLeft.setPosition(0.91);
                    flipRight.setPosition(0.91);
                }
                if (gamepad2.right_trigger>0.1) {
                    rolling.setPower(0.95);

                }else if (gamepad2.left_trigger>0.1) {
                    rolling.setPower(-0.95);

                } else if (gamepad2.left_trigger<0.1 && gamepad2.right_trigger<0.1) {
                    rolling.setPower(0);
                }

                intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                if (gamepad2.right_bumper) {
                    intakeSlide.setPower(0.5);

                }else if (gamepad2.left_bumper) {
                    intakeSlide.setPower(-0.5);

                } else if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
                    intakeSlide.setPower(0);
                }



                if (gamepad1.dpad_up){

                    deposoitRight.setPosition(1);
                    depositLeft.setPosition(1);
                } else if (gamepad1.dpad_down) {

                    depositLeft.setPosition(0.5);
                    deposoitRight.setPosition(0.5);
                }
                else if (gamepad1.dpad_left) {

                    depositLeft.setPosition(0);
                    deposoitRight.setPosition(0);
                }

                //flipRight.setPosition(gamepad2.left_stick_y);

                intakeMotorRight.setDirection(DcMotorImplEx.Direction.REVERSE);
                intakeMotorLeft.setPower(gamepad1.right_stick_y);
                intakeMotorRight.setPower(gamepad1.right_stick_y);





                //drive.updatePoseEstimate();
                telemetry.addData("Flip Right value", flipRight.getPosition());
                telemetry.addData("Flip Left value", flipLeft.getPosition());
                telemetry.addData("Flip Left value", rolling.getPower());
                telemetry.addData("slidePosLeft", intakeMotorLeft.getCurrentPosition());
                telemetry.addData("slidePosRight", intakeMotorRight.getCurrentPosition());
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
              //  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
