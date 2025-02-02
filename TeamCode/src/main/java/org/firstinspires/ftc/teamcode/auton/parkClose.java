package org.firstinspires.ftc.teamcode.auton;



import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Assembly;


@Autonomous(name = "Park for close basket", group = "Test")
public class parkClose extends LinearOpMode {

    MecanumDrive drive;

    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        runtime.reset();

        Action traj1 = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeToLinearHeading(new Vector2d(5, 20), 0)
                .stopAndAdd(new SleepAction(1))
                .strafeToLinearHeading(new Vector2d(55,20), 0)
                .stopAndAdd(new SleepAction(1))
                .strafeToLinearHeading(new Vector2d(55,10), 0)
                .build();






        while (opModeInInit()){
        }
        waitForStart();

        Actions.runBlocking(new SequentialAction(
                traj1
        ));

    }
}
