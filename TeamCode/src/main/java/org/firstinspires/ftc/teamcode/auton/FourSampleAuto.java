package org.firstinspires.ftc.teamcode.auton;



import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Assembly;

//import org.firstinspires.ftc.teamcode.teleop.Deposit;
//import org.firstinspires.ftc.teamcode.teleop.PIDController;
//import org.firstinspires.ftc.teamcode.teleop.Rolling;


@Autonomous(name = "Auton LM2 + Park", group = "16481-IntoTheDeep")
public class FourSampleAuto extends LinearOpMode {

   Assembly assembly;


    MecanumDrive drive;


    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        assembly = new Assembly(hardwareMap);

        runtime.reset();

        Action traj = drive.actionBuilder(new Pose2d(0,0,0))
                .stopAndAdd(new SequentialAction(
                        assembly.clawClose(),
                        assembly.anglePitch(Assembly.PitchPosition.DOWN),
                        new SleepAction(2),
                        assembly.extendSlide(Assembly.SlidesPosition.DOWN),
                        new SleepAction(2)
                ))
                .strafeToLinearHeading(new Vector2d(10, 40), 325)
                .stopAndAdd(new SequentialAction(
                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
                        new SleepAction(1000),
                        assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                        new SleepAction(500),
                        assembly.flipUp(),
                        new SleepAction(1000),
                        assembly.clawOpen(),
                        new SleepAction(1000),
                        assembly.flipDown(),
                        assembly.extendSlide(Assembly.SlidesPosition.DOWN),
                        new SleepAction(1000),
                        assembly.anglePitch(Assembly.PitchPosition.DOWN)
                ))
                .strafeToLinearHeading(new Vector2d(23,31),0)
                .stopAndAdd(new SequentialAction(
                        assembly.flipDown(),
                        new SleepAction(1000),
                        assembly.clawClose()
                ))
                .strafeToLinearHeading(new Vector2d(10, 40), 325)
                .stopAndAdd(new SequentialAction(
                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
                        new SleepAction(1000),
                        assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                        new SleepAction(1000),
                        assembly.flipUp(),
                        new SleepAction(1000),
                        assembly.clawOpen(),
                        new SleepAction(1000)
                ))
                .build();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                traj
                        ),
                        telemetryPacket -> {
                            assembly.update();
                            // telemtry here
                            telemetry.update();
                            return opModeIsActive();
                        }
                )
        );

    }
}