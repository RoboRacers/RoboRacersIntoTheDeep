package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.PIDController;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Assembly;

@TeleOp(name = "LM2 Subsystems", group = "Test")
public class LM2subsystems extends LinearOpMode {
    //Pitch Stuff

    Assembly assembly;

ElapsedTime elapsedTime;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {



        assembly = new Assembly(hardwareMap);
//        slidesMotor.setDirection(DcMotorImplEx.Direction.REVERSE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();





        elapsedTime = new ElapsedTime();

//        pitchControl.setErrorTolerance(18);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));



        resetRuntime();

        while (opModeInInit()) {
            assembly.flipUp();
            assembly.anglePitch(300);
            assembly.update();
        }



        resetRuntime();

        while (!isStopRequested()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();


            if (gamepad1.triangle) { //y
                assembly.anglePitch(1010);

                Thread.sleep(1500);
                assembly.flipMid();
                assembly.slidesManual(1650);

//                target2 = 1010; //90deg + little more
////                sleep(1000);
////                wait(1000);
//                target=1650;
//                flipPos = 0.525;
//
            } else if (gamepad1.cross) { // a
//                target2 = 300;
////                sleep(1000);
////                wait(1000);
//                target= 400;
//                flipPos = 0.355; // Down so that we can go into middle thing
                assembly.anglePitch(300);
                Thread.sleep(1500);
                assembly.flipMid();
                assembly.slidesManual(400);

            } else if (gamepad1.circle) { // b
//                flipPos = 0.111;
////                wait(1000);
//                target = 450;
//                target2 = 270;  // Pick up with claw down

                assembly.anglePitch(270);
                Thread.sleep(1500);
                assembly.slidesManual(450);
                assembly.flipDown();

            }else if (gamepad1.square) { // x
//                target2 = 500;   // no function

                assembly.anglePitch(500);

            }

            if(gamepad1.right_trigger>0.1){
               assembly.extendSlide(Assembly.SlidesPosition.MANUALUP);//extend
            } else if (gamepad1.left_trigger>0.1) {
                assembly.extendSlide(Assembly.SlidesPosition.MANUALDOWN); //retract
            }

//            //ALWAYS MULTIPLY THE RIGHT FLIP OR DOS BY 0.95 TO MAKE IT SYNC WITH THE LEFT DEPOSIT OR UNO
//
//
            if (gamepad1.dpad_up){
                assembly.rotateClaw(0.1);
            } else if (gamepad1.dpad_down) {
                assembly.rotateClaw(0.5);
//                rotateClaw.setPosition(0.6);
            } else if (gamepad1.dpad_left) {
                assembly.rotateClaw(0.3);
//                rotateClaw.setPosition(0.35);
            } else if (gamepad1.dpad_right) {
                assembly.rotateClaw(0.9);
            }

            if (gamepad1.right_bumper){
                assembly.clawClose();//close
            }else if(gamepad1.left_bumper){
                assembly.clawOpen(); //open
            }

            assembly.update();

//            telemetry.addData("Slides Power", assembly.getPower());
//            telemetry.addData("slides Pos", assembly.getCurrentPosition());
//            telemetry.addData("Target value Pitch", target);
//            telemetry.addData("Feedforward", feedforward);
//            telemetry.addData("PID Values", pid);
//            telemetry.addData("Pitch Motor Position", assembly.pitchMotor.getCurrentPosition());
//            telemetry.addData("Pitch Motor Power", assembly.pitchMotor.getPower());
//            telemetry.addData("Pitch Motor Angle", (assembly.pitchMotor.getCurrentPosition() - 20 ) * ticksToDegrees);
//            telemetry.addData("Uno pos", uno.getPosition());
//            telemetry.addData("Dos pos", dos.getPosition());
            telemetry.update();

        }
    }
}
