package org.firstinspires.ftc.teamcode.auton;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.PIDController;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Slides;

//import org.firstinspires.ftc.teamcode.teleop.Deposit;
//import org.firstinspires.ftc.teamcode.teleop.PIDController;
//import org.firstinspires.ftc.teamcode.teleop.Rolling;


@Autonomous(name = "Auton LM2 + Park", group = "16481-IntoTheDeep")
public class bet extends LinearOpMode {

   Slides assembly;


    MecanumDrive drive;


    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        // MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));



        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
assembly = new Slides(hardwareMap);

        runtime.reset();
        waitForStart();

        Action traj1 = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeToLinearHeading(new Vector2d(10, 40), 325)
                .build();

        Action traj2 = drive.actionBuilder(new Pose2d(10,40, 320))
                .strafeToLinearHeading(new Vector2d(23,31),0)
                .build();

        Action traj3 = drive.actionBuilder(new Pose2d(23, 31, 0))
                .strafeToLinearHeading(new Vector2d(10, 40), 325)
                .build();

        Action traj4 = drive.actionBuilder(new Pose2d(46, 10, 90))
                .splineToConstantHeading(new Vector2d(60, 0), 90)
                .build();

        Actions.runBlocking(new SequentialAction(

                assembly.clawClose(),
                assembly.anglePitch(Slides.PitchPosition.DOWN),
                new SleepAction(2),
                assembly.extendSlide(Slides.SlidesPosition.DOWN),
                new SleepAction(2),
                traj1,
                assembly.anglePitch(Slides.PitchPosition.HIGH),
                new SleepAction(1000),
                assembly.extendSlide(Slides.SlidesPosition.HIGH),
                new SleepAction(500),
                assembly.flipUp(),
                new SleepAction(1000),
                assembly.clawOpen(),
                new SleepAction(1000),
                assembly.flipDown(),
                assembly.extendSlide(Slides.SlidesPosition.DOWN),
                new SleepAction(1000),
                assembly.anglePitch(Slides.PitchPosition.DOWN),
                traj2,
                assembly.flipDown(),
                new SleepAction(1000),
                assembly.clawClose(),
                traj3,
                assembly.anglePitch(Slides.PitchPosition.HIGH),
                new SleepAction(1000),
                assembly.extendSlide(Slides.SlidesPosition.HIGH),
                new SleepAction(1000),
                assembly.flipUp(),
                new SleepAction(1000),
                assembly.clawOpen(),
                new SleepAction(1000)

        ));

        while (opModeIsActive()) {



            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            //  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }
    }
}