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
import com.qualcomm.robotcore.hardware.Gamepad;
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
    ServoImplEx flipRight;
    ServoImplEx flipLeft;
    ServoImplEx v4bServo;

    @Override
    public void runOpMode() throws InterruptedException {

        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");

        flipRight = hardwareMap.get(ServoImplEx.class, "flipRight");
        flipLeft = hardwareMap.get(ServoImplEx.class, "flipLeft");
        v4bServo = hardwareMap.get(ServoImplEx.class, "v4bServo");

        while (opModeInInit()) {
        }

        waitForStart();

        while (!isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad1.a) {
                flipRight.setPosition(0.90);
                flipLeft.setPosition(0.90);
                v4bServo.setPosition(0.3);
            } else if (gamepad1.y) {
                flipRight.setPosition(0.30);
                flipLeft.setPosition(0.30);
                v4bServo.setPosition(0.39);
            }

            intakeMotor.setPower(-gamepad1.left_stick_y);
            telemetry.update();

        }
    }
}
