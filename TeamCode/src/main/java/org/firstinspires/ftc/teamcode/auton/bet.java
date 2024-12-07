package org.firstinspires.ftc.teamcode.auton;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.PIDController;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Slides;


@Autonomous(name = "1+1 bet", group = "Test")
public class bet extends LinearOpMode {

    MecanumDrive drive;
    Slides assembly;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        while (!isStopRequested()){

        }

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        assembly = new Slides(hardwareMap);
        runtime.reset();

        Action traj1 = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeToLinearHeading(new Vector2d(10, 40), 325)
                .build();

        Action traj2 = drive.actionBuilder(new Pose2d(10,40, 320))
                .strafeToLinearHeading(new Vector2d(23,31),0)
                .build();

        Action traj3 = drive.actionBuilder(new Pose2d(23, 31, 0))
                .strafeToLinearHeading(new Vector2d(10, 40), 325)
                .build();

        Action traj4 = drive.actionBuilder(new Pose2d(23, 31, 0))
                .strafeToLinearHeading(new Vector2d(10, 40), 325)
                .build();
        Actions.runBlocking(new SequentialAction(
                assembly.clawClose(),
                traj1,
                assembly.anglePitch(Slides.PitchPosition.HIGH),
                new SleepAction(1000),
                assembly.extendSlide(Slides.SlidesPosition.HIGH),
                new SleepAction(1000),
                assembly.flipUp(),
                new SleepAction(1000),
                assembly.clawOpen(),
                new SleepAction(1000),
                traj2,

                traj3

        ));

    }
}