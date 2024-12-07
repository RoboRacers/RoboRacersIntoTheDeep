//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorImplEx;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.modules.PIDController;
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.robot.Slides;
//
//@TeleOp(name = "LM2 One Driver", group = "Test")
//public class OneDriverSubsystems extends LinearOpMode {
//    //Pitch Stuff
//   Slides robot;
//
//    MecanumDrive drive;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//
//
//
////        pitchControl.setErrorTolerance(18);
//
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//        robot = new Slides(hardwareMap);
//
//
//        while (opModeInInit()) {
//            flipPos = 0.9;
//            pitchControl.setCoefficients(kP, kI, kD);
//            uno.setPosition(flipPos);
//            dos.setPosition(flipPos * 0.94);
//            target2 = 250;
//
//
//            pitchControl.setSetpoint(target2);
//
//
//            double feedforward2 = kG2 * ((slidesMotor.getCurrentPosition()) * ticksToInches) + 0;
//            double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
//            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
//            pitchMotor.setPower(feedforward3 + pid + feedforward2);
//
//
//        }
//
//
//
//        while (!isStopRequested()) {
//            drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x
//                    ),
//                    -gamepad1.right_stick_x
//            ));
//            drive.updatePoseEstimate();
//            pitchControl.setCoefficients(kP, kI, kD);
//            slidesControl.setCoefficients(kP2, kI2, kD2);
//
//            if (gamepad1.triangle) { //y
//                target2 = 1010; //90deg + little more
////                sleep(1000);
////                wait(1000);
//                target=1650;
//                flipPos = 0.525;
//            } else if (gamepad1.cross) { // a
//                target2 = 300;
////                sleep(1000);
////                wait(1000);
//                target= 400;
//                flipPos = 0.355; // Down so that we can go into middle thing
//            } else if (gamepad1.circle) { // b
//                flipPos = 0.111;
////                wait(1000);
//                target2 = 320;  // Pick up with claw down
//            }else if (gamepad1.square) { // x
//                target2 = 500;   // no function
//            }
//
//            if(gamepad1.right_trigger>0.1){
//                target+= 70;//extend
//            } else if (gamepad1.left_trigger>0.1) {
//                target-= 70; //retract
//            }else{
//                target = target;
//            }
//
//            pitchControl.setSetpoint(target2);
//
//            slidesControl.setSetpoint(target);
//            double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
//            double feedforward2 = kG2 * (slidesMotor.getCurrentPosition() * ticksToInches) + 0;
//            double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
//            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
//            double pid2 = slidesControl.calculate(slidesMotor.getCurrentPosition());
//
//
//            pitchMotor.setPower(feedforward3 + pid + feedforward2);
//            slidesMotor.setPower(-(pid2 + feedforward));
//
//            //ALWAYS MULTIPLY THE RIGHT FLIP OR DOS BY 0.95 TO MAKE IT SYNC WITH THE LEFT DEPOSIT OR UNO
//
//
//            uno.setPosition(flipPos);
//            dos.setPosition(flipPos * 0.94);
//
//
//            if (gamepad1.dpad_up){
//                rotateClaw.setPosition(0.05);
//            } else if (gamepad1.dpad_down) {
//                rotateClaw.setPosition(0.6);
//            } else if (gamepad1.dpad_left) {
//                rotateClaw.setPosition(0.35);
//            } else if (gamepad1.dpad_right) {
//                rotateClaw.setPosition(0.96);
//            }
//
//            if (gamepad1.right_bumper){
//                claw.setPosition(0.70); //close
//            }else if(gamepad1.left_bumper){
//                claw.setPosition(0.425); //open
//            }
//
//
//            telemetry.addData("Slides Power", slidesMotor.getPower());
//            telemetry.addData("slides Pos", slidesMotor.getCurrentPosition());
//            telemetry.addData("Target value Pitch", target);
//            telemetry.addData("Feedforward", feedforward);
//            telemetry.addData("PID Values", pid);
//            telemetry.addData("Pitch Motor Position", pitchMotor.getCurrentPosition());
//            telemetry.addData("Pitch Motor Power", pitchMotor.getPower());
//            telemetry.addData("Pitch Motor Angle", (pitchMotor.getCurrentPosition() - 20 ) * ticksToDegrees);
//            telemetry.addData("Uno pos", uno.getPosition());
//            telemetry.addData("Dos pos", dos.getPosition());
//            telemetry.update();
//
//        }
//    }
//}
