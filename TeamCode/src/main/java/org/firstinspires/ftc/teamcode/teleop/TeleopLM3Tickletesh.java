package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.AssemblyShrekster;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Teleop LM3", group = "0000-Final")
public class TeleopLM3Tickletesh extends LinearOpMode {

    ElapsedTime elapsedTime;

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    MecanumDrive drive;
    AssemblyShrekster assembly;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {

        assembly = new AssemblyShrekster(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        while (opModeInInit()) {
            assembly.pitchTarget = 0;
            assembly.update();
        }

        waitForStart();

        while (!isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();

            if(gamepad1.triangle){
                assembly.SmoothDepositUp();
            }

            if(gamepad1.cross){
                assembly.SmoothDepositDown();
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
            dash.sendTelemetryPacket(packet);

            previousGamepad1 = gamepad1;
            previousGamepad2 = gamepad2;

            assembly.update();

            // Pitch
            telemetry.addData("Pitch Motor Position", assembly.pitchMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", assembly.pitchMotor.getPower());
            telemetry.addData("Pitch Motor Angle", assembly.getPitchAngle());
            telemetry.addData("Pitch Target", AssemblyShrekster.pitchTarget);
            // Slides
            telemetry.addData("Slides Motor Position", assembly.slidesMotor.getCurrentPosition());
            telemetry.addData("Slides Motor Power", assembly.slidesMotor.getPower());
            telemetry.addData("Slides Target", AssemblyShrekster.slideTarget);
            telemetry.update();

        }
    }
}