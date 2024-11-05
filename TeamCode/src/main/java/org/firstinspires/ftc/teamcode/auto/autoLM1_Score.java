package org.firstinspires.ftc.teamcode.auto;



import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Deposit;
import org.firstinspires.ftc.teamcode.teleop.PIDController;
import org.firstinspires.ftc.teamcode.teleop.Rolling;


@Autonomous(name = "LM1 Auton Park", group = "16481-IntoTheDeep")
public class autoLM1_Score extends LinearOpMode {

    MecanumDrive drive;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


// MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        runtime.reset();
waitForStart();

        while (opModeIsActive() && runtime.seconds() <4) {

        drive.leftBack.setPower(-0.5);

            drive.rightBack.setPower(0.5);

            drive.rightFront.setPower(-0.5);

            drive.leftFront.setPower(0.5);

                    telemetry.update();

TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
//  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }
                drive.leftBack.setPower(0);
//fixcodeplz
//tyjava
        drive.rightBack.setPower(0);

        drive.rightFront.setPower(0);

        drive.leftFront.setPower(0);

    }
}