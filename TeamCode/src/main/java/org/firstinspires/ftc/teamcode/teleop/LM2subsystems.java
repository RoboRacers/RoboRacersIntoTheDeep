//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorImplEx;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.modules.PIDController;
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.robot.Assembly;
//
//import java.util.ArrayList;
//import java.util.List;
//@Disabled
//@TeleOp(name = "LM2 Subsystems", group = "Test")
//public class LM2subsystems extends LinearOpMode {
//
//    Assembly assembly;
//
//    ElapsedTime elapsedTime;
//
//    Gamepad currentGamepad1 = new Gamepad();
//    Gamepad currentGamepad2 = new Gamepad();
//
//    Gamepad previousGamepad1 = new Gamepad();
//    Gamepad previousGamepad2 = new Gamepad();
//    MecanumDrive drive;
//
//
//    public boolean intakeToggle = false;  // Motor starts off
//    public boolean intakeToggle2 = false;  // Motor starts off
//    public double triggerThreshold = 0.2;
//
//
//
//    private FtcDashboard dash = FtcDashboard.getInstance();
//    private List<Action> runningActions = new ArrayList<>();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        assembly = new Assembly(hardwareMap);
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//        while (opModeInInit()) {
//            runningActions.add(
//                    new SequentialAction(
//                            assembly.anglePitch(300),
//                            assembly.flipUp()
//                    )
//            );
//            assembly.update();
//        }
//
//        waitForStart();
//
//        Gamepad prevGamepad1 = new Gamepad();  // to keep track of previous gamepad state
//
//        while (!isStopRequested()) {
//            TelemetryPacket packet = new TelemetryPacket();
//
//            // updated based on gamepads
//            if (gamepad1.triangle) { //y
//                runningActions.add(
//                        new SequentialAction(
//                                assembly.anglePitch(1010),
//                                new SleepAction(0.5),
//                                assembly.extendSlide(Assembly.SlidesPosition.HIGH),
//                            new SleepAction(0.5),
//                                assembly.flipMid()
//                        )
//                );
//            } else if (gamepad1.cross) { // a
//                runningActions.add(
//                        new SequentialAction(
//                                assembly.anglePitch(300),
//                                new SleepAction(0.5),
//                                assembly.extendSlide(Assembly.SlidesPosition.DOWN),
//                                new SleepAction(0.5),
//                                assembly.flipMid()
//                        )
//                );
//            }
//            else if (gamepad1.circle) { // b
//                runningActions.add(new SequentialAction(
//                        assembly.flipDown(),
//                        new SleepAction(0.5),
//                        assembly.anglePitch(270)
//
////                        assembly.extendSlide(400)
//                ));
//            }else if (gamepad1.square) { // x
//                runningActions.add(new SequentialAction(
//                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
//                        new SleepAction(0.5),
//                        assembly.extendSlide(Assembly.SlidesPosition.DOWN),
//                        new SleepAction(0.5),
//                        assembly.flipUp()
//                ));
//            }
//
//
//
//
//
//// Threshold for trigger detection (e.g., 0.5 is the usual threshold for triggers)
//
//// Rising edge detection for the left trigger (detect when it's pressed)
//            if (currentGamepad1.left_trigger > triggerThreshold && previousGamepad1.left_trigger <= triggerThreshold) {
//                // Rising edge detected, toggle the intake motor
//                intakeToggle = !intakeToggle;
//            }
//
//// Rising edge detection for the right trigger (detect when it's pressed)
//            if (currentGamepad1.right_trigger > triggerThreshold && previousGamepad1.right_trigger <= triggerThreshold) {
//                // Rising edge detected, toggle the intake motor
//                intakeToggle2 = !intakeToggle2;
//            }
//
//// Else condition to handle when trigger is released
//// Check for falling edge (trigger released) to reset intakeToggle, if desired
//            else if (currentGamepad1.left_trigger <= triggerThreshold && previousGamepad1.left_trigger > triggerThreshold) {
//                // Falling edge detected for the left trigger (trigger released)
//                // Optionally change intakeToggle based on the release, for example:
//                intakeToggle = false; // Resetting intakeToggle when the trigger is released
//            }
//
//            else if (currentGamepad1.right_trigger <= triggerThreshold && previousGamepad1.right_trigger > triggerThreshold) {
//                // Falling edge detected for the right trigger (trigger released)
//                // Optionally change intakeToggle based on the release, for example:
//                intakeToggle2 = false; // Resetting intakeToggle when the trigger is released
//            }
//
//// Using the toggle variable to control the robot's intake motor.
//            if (intakeToggle) {
//                runningActions.add(
//                        new SequentialAction(
//                                assembly.extendSlide(Assembly.SlidesPosition.MANUALUP)
//                        )//extend
//                );            }
//            else if (intakeToggle2) {
//                runningActions.add(
//                        new SequentialAction(
//                                assembly.extendSlide(Assembly.SlidesPosition.MANUALDOWN)
//                        )//retract
//                );
//            }
//
//// Update previous gamepad values for next cycle (to detect rising or falling edges correctly)
//            previousGamepad1.left_trigger = currentGamepad1.left_trigger;
//            previousGamepad1.right_trigger = currentGamepad1.right_trigger;
//
//
//
//
//            if(gamepad1.right_trigger>0.1 && prevGamepad1.right_trigger <= 0.1){
//                // This only runs ONE loop of the action, so that we don't end up incrementing the slides position multiple times
//                assembly.extendSlide(Assembly.SlidesPosition.MANUALUP).run(packet);
//            } else if (gamepad1.left_trigger>0.1 && prevGamepad1.left_trigger <= 0.1){
//                // same for this one
//                assembly.extendSlide(Assembly.SlidesPosition.MANUALDOWN).run(packet);
//            }
//
//            if (gamepad1.dpad_up){
//                runningActions.add(
//                        new SequentialAction(
//                                assembly.rotateClaw(0.1)));
//            } else if (gamepad1.dpad_down) {
//                runningActions.add(
//                        new SequentialAction(
//                                assembly.rotateClaw(0.5)));
////                rotateClaw.setPosition(0.6);
//            } else if (gamepad1.dpad_left) {
//                runningActions.add(
//                        new SequentialAction(
//                                assembly.rotateClaw(0.3)));
////                rotateClaw.setPosition(0.35);
//            } else if (gamepad1.dpad_right) {
//                runningActions.add(
//                        new SequentialAction(
//                                assembly.rotateClaw(0.9)));
//            }
//
//            if (gamepad1.right_bumper){
//                runningActions.add(
//                        new SequentialAction(
//                                assembly.clawClose()));//close
//            }else if(gamepad1.left_bumper){
//                runningActions.add(
//                        new SequentialAction(
//                                assembly.clawOpen())); //open
//            }
//
//            drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x
//                    ),
//                    -gamepad1.right_stick_x
//            ));
//            drive.updatePoseEstimate();
//
//            // update running actions
//            List<Action> newActions = new ArrayList<>();
//            for (Action action : runningActions) {
//                action.preview(packet.fieldOverlay());
//                if (action.run(packet)) {
//                    newActions.add(action);
//                }
//            }
//            runningActions = newActions;
//            dash.sendTelemetryPacket(packet);
//
//            prevGamepad1 = gamepad1;
//
//            assembly.update();
////            telemetry.addData("Slides Power", assembly.getPower());
////            telemetry.addData("slides Pos", assembly.getCurrentPosition());
////            telemetry.addData("Target value Pitch", target);
////            telemetry.addData("Feedforward", feedforward);
////            telemetry.addData("PID Values", pid);
////            telemetry.addData("Pitch Motor Position", assembly.pitchMotor.getCurrentPosition());
////            telemetry.addData("Pitch Motor Power", assembly.pitchMotor.getPower());
////            telemetry.addData("Pitch Motor Angle", (assembly.pitchMotor.getCurrentPosition() - 20 ) * ticksToDegrees);
////            telemetry.addData("Uno pos", uno.getPosition());
////            telemetry.addData("Dos pos", dos.getPosition());
//            telemetry.update();
//
//        }
//    }
//}
