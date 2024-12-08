package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "LM2 Subsystems", group = "Test")
public class LM2subsystems extends LinearOpMode {
    //Pitch Stuff

    Assembly assembly;

    ElapsedTime elapsedTime;
    MecanumDrive drive;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

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
            runningActions.add(
                    new SequentialAction(
                            assembly.anglePitch(300),
                            assembly.flipUp()
                    )
            );
            assembly.update();
        }



        resetRuntime();

        while (!isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            // updated based on gamepads

            if (gamepad1.triangle) { //y
                runningActions.add(
                        new SequentialAction(
                                assembly.anglePitch(1010),
                                new SleepAction(0.5),
                                assembly.flipMid()
                        )
                );
            } else if (gamepad1.cross) { // a
                runningActions.add(
                        new SequentialAction(
                                assembly.anglePitch(300),
                                new SleepAction(0.5),
                                assembly.flipMid()
                        )
                );
            }
            else if (gamepad1.circle) { // b
                runningActions.add(new SequentialAction(
                        assembly.flipDown(),
                        new SleepAction(0.5),
                        assembly.anglePitch(270)
//                        assembly.extendSlide(400)
                ));


            }else if (gamepad1.square) { // x
                runningActions.add(new SequentialAction(
                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
                        new SleepAction(0.5),
                        assembly.extendSlide(100),
                        assembly.flipUp()
                ));
            }
            if(gamepad1.right_trigger>0.1){
                runningActions.add(
                        new SequentialAction(
                                assembly.extendSlide(Assembly.SlidesPosition.MANUALUP)
                        )//extend
                );
            } else if (gamepad1.left_trigger>0.1) {
                runningActions.add(
                        new SequentialAction(
                                assembly.extendSlide(Assembly.SlidesPosition.MANUALDOWN)
                        )//retract
                );
            }
            if (gamepad1.dpad_up){
                runningActions.add(
                        new SequentialAction(
                                assembly.rotateClaw(0.1)));
            } else if (gamepad1.dpad_down) {
                runningActions.add(
                        new SequentialAction(
                                assembly.rotateClaw(0.5)));
//                rotateClaw.setPosition(0.6);
            } else if (gamepad1.dpad_left) {
                runningActions.add(
                        new SequentialAction(
                                assembly.rotateClaw(0.3)));
//                rotateClaw.setPosition(0.35);
            } else if (gamepad1.dpad_right) {
                runningActions.add(
                        new SequentialAction(
                                assembly.rotateClaw(0.9)));
            }

            if (gamepad1.right_bumper){
                runningActions.add(
                        new SequentialAction(
                                assembly.clawClose()));//close
            }else if(gamepad1.left_bumper){
                runningActions.add(
                        new SequentialAction(
                                assembly.clawOpen())); //open
            }

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            //
            dash.sendTelemetryPacket(packet);

//                assembly.anglePitch(500).run(new TelemetryPacket());




//            //ALWAYS MULTIPLY THE RIGHT FLIP OR DOS BY 0.95 TO MAKE IT SYNC WITH THE LEFT DEPOSIT OR UNO
//


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
