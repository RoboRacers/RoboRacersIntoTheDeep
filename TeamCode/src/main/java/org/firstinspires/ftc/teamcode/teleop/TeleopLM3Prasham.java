package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Assembly;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Teleop LM3 Prasham :]", group = "0000-Final")
public class TeleopLM3Prasham extends LinearOpMode {

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    MecanumDrive drive;
    Assembly assembly;

    private FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        assembly = new Assembly(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        while (opModeInInit()) {
            assembly.setPitchTarget(Assembly.PITCH_MID_POSITION);
            assembly.update();
        }

        waitForStart();

        while (!isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();

            // Pitch presets

            if (gamepad1.dpad_up) {
                assembly.setPitchTarget(Assembly.PITCH_HIGH_POSITION);
            } else if (gamepad1.dpad_down && (assembly.slideTarget < 45)) {
                assembly.setPitchTarget(Assembly.PITCH_LOW_POSITION);
            } else if (gamepad1.dpad_right && (assembly.slideTarget < 45)){
                assembly.setPitchTarget(Assembly.PITCH_MID_POSITION);
            }



            // Slide Presets
            if (gamepad1.triangle) {
                assembly.slideTarget = Assembly.SLIDES_HIGH_POSITION;
            } else if (gamepad1.cross) {
                assembly.slideTarget = Assembly.SLIDES_LOW_POSITION;
            }

            if (gamepad1.right_bumper){
                assembly.claw.setPosition(0.35);
            }else if(gamepad1.left_bumper){
                assembly.claw.setPosition(0.1);
            }

            // Flip
            if (gamepad1.circle) {
                assembly.flipRight.setPosition(0.130 * 0.94);
                assembly.flipLeft.setPosition(0.130);
            } else if (gamepad1.square) {
                assembly.flipRight.setPosition(0.5 * 0.94);
                assembly.flipLeft.setPosition(0.5);
            }


            if (gamepad1.right_trigger > 0.1){
                assembly.rotateClaw.setPosition(0.116);
            }else if(gamepad1.left_trigger > 0.1){
                assembly.rotateClaw.setPosition(0.483);
            }

            if (gamepad2.right_bumper){
                assembly.slideTarget = Assembly.SLIDES_MID_POSITION;
                telemetry.addLine("test");
            } else if(gamepad2.left_bumper){
                assembly.slideTarget = Assembly.SLIDES_AUTO_POSITION;
            }

            // Rotate
            /*
            if (gamepad1.dpad_right && !previousGamepad1.dpad_right) {
                assembly.rotateClaw.setPosition(
                        assembly.rotateClaw.getPosition() + 0.1
                );
            } else if (gamepad1.dpad_left && !previousGamepad1.dpad_left) {
                assembly.rotateClaw.setPosition(
                        assembly.rotateClaw.getPosition() - 0.1
                );
            }

             */


            // Claw
            /*
            if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
                assembly.toggleClaw();
            }

             */

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();


            dash.sendTelemetryPacket(packet);
            previousGamepad1 = gamepad1;
            previousGamepad2 = gamepad2;

            assembly.update();

            // Pitch
            telemetry.addData("Pitch Motor Position", assembly.pitchMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", assembly.pitchMotor.getPower());
            telemetry.addData("Pitch Motor Angle", assembly.getPitchAngle());
            telemetry.addData("Pitch Target", Assembly.pitchTarget);
            // Slides
            telemetry.addData("Slides Motor Position", assembly.slidesMotor.getCurrentPosition());
            telemetry.addData("Slides Motor Power", assembly.slidesMotor.getPower());
            telemetry.addData("Slides Target", assembly.slideTarget);
            telemetry.update();

        }
    }
}
