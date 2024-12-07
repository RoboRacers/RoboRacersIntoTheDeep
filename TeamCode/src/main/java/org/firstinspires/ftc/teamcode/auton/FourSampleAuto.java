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


@Autonomous(name = "Bet", group = "16481-IntoTheDeep")
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
                        assembly.flipDown(),
                        assembly.anglePitch(Assembly.PitchPosition.DOWN),
//                        new SleepAction(2),
                        assembly.extendSlide(Assembly.SlidesPosition.DOWN)

//                        new SleepAction(2)
                ))
                .splineTo(new Vector2d(7, 36), Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
                        new SleepAction(2),
                        assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                        new SleepAction(1),
                        assembly.flipMid(),
                        new SleepAction(1),
                        assembly.clawOpen(),
                        new SleepAction(2000)
                ))
                .splineTo(new Vector2d(9, 40), Math.toRadians(-45))
                        .stopAndAdd(new SequentialAction(
                                new SleepAction(4),
                                assembly.flipDown(),
                                new SleepAction(1),
//                                assembly.flipDown(),
                                assembly.extendSlide(Assembly.SlidesPosition.DOWN),
                                new SleepAction(2),
                                assembly.anglePitch(Assembly.PitchPosition.DOWN)
                        ))

                .strafeToLinearHeading(new Vector2d(23,31),0)
                .stopAndAdd(new SequentialAction(
                        assembly.flipDown(),
                        new SleepAction(1),
                        assembly.clawClose()
                ))
                .strafeToLinearHeading(new Vector2d(10, 40), 325)
                .stopAndAdd(new SequentialAction(
                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
                        new SleepAction(1),
                        assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                        new SleepAction(1),
                        assembly.flipUp(),
                        new SleepAction(1),
                        assembly.clawOpen(),
                        new SleepAction(1)
                ))
                .build();
        while (opModeInInit()){
            assembly.clawClose();
                    assembly.flipMid();
                    assembly.anglePitch(Assembly.PitchPosition.DOWN);
                    new SleepAction(2);
                    assembly.extendSlide(Assembly.SlidesPosition.DOWN);
                    new SleepAction(2);
        }


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