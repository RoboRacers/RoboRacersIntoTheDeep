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


@Autonomous(name = "BetLess", group = "16481-IntoTheDeep")
public class SpecimenAuton extends LinearOpMode {

   Assembly assembly;


    MecanumDrive drive;


    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        assembly = new Assembly(hardwareMap);

        runtime.reset();

        Action traj = drive.actionBuilder(new Pose2d(0,0,Math.toRadians(180)))
                // Init Position
                .stopAndAdd(new SequentialAction(
                        assembly.rotateClaw(0.48),
                        assembly.clawClose(),
                        assembly.flipMid(),
                        assembly.anglePitch(Assembly.PitchPosition.DOWN),
//                        new SleepAction(2),
                        assembly.extendSlide(350),
                        new SleepAction(1),
                        assembly.anglePitch(Assembly.PitchPosition.HIGH)
//                        new SleepAction(2)
                ))
                .splineToLinearHeading(new Pose2d(10,10,Math.toRadians(180)),Math.toRadians(180))

//           above or this one     .splineToLinearHeading(new Pose2d(11,34,Math.toRadians(0)),Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        // To score Preload in first basket
                        new SleepAction(1),
                        assembly.extendSlide(Assembly.SlidesPosition.MID),
                        new SleepAction(1),
                        assembly.flipUp()
                ))
                .splineToLinearHeading(new Pose2d(22,10,Math.toRadians(180)),Math.toRadians(180))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(1),
                        assembly.extendSlide(Assembly.SlidesPosition.DOWN),
                        new SleepAction(0.25),
                        assembly.clawOpen()
                ))

                // To pick up 1st sample(right side) on floor
//                .splineToLinearHeading(new Vector2d(12,10,Math.toRadians(180)),Math.toRadians(180))
//                .strafeToConstantHeading(new Vector2d(6, -32), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(
                        assembly.extendSlide(100),
                        new SleepAction(1),
                        assembly.anglePitch(1150)
                ))
                .strafeToLinearHeading(new Vector2d(2, -32), Math.toRadians(0))
                // To drop 1st sample from floor into high basket
                .stopAndAdd(new SequentialAction(
                        assembly.clawClose(),
                        new SleepAction(1000),
                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
                        new SleepAction(1),
                        assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                        new SleepAction(1),
                        assembly.flipMid(),
                        new SleepAction(1),
                        assembly.clawOpen(),
                        new SleepAction(0.5),
                        assembly.flipDown(),
                        assembly.extendSlide(450),
                        new SleepAction(0.5),
                        assembly.anglePitch(Assembly.PitchPosition.DOWN)
                ))
                // To pickup 2nd sample(middle) from the floor
                .strafeToLinearHeading(new Vector2d(13, 41), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(
                        assembly.anglePitch(450),
                        assembly.extendSlide(Assembly.SlidesPosition.MID),
                        new SleepAction(0.5),
                        assembly.flipDown(),
                        new SleepAction(1),
                        assembly.anglePitch(200),
                        new SleepAction(1),
                        assembly.clawClose()
                ))
                // To drop 2nd sample into high basket
                .strafeToLinearHeading(new Vector2d(7, 36), Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
                        new SleepAction(1),
                        assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                        new SleepAction(1),
                        assembly.flipMid(),
                        new SleepAction(1),
                        assembly.clawOpen(),
                        new SleepAction(1000),
                        assembly.flipDown(),
                        new SleepAction(1),
                        assembly.extendSlide(450),
                        new SleepAction(2),
                        assembly.anglePitch(Assembly.PitchPosition.DOWN),
                        new SleepAction(3)

                ))
                // To pickup 3rd sample(middle) from the floor
                .strafeToLinearHeading(new Vector2d(25, 55), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(
                        assembly.extendSlide(475),
                        assembly.anglePitch(135),
                        new SleepAction(1),
                        assembly.rotateClaw(0.9),
                        new SleepAction(2),
                        assembly.clawClose(),
                        new SleepAction(1),
                        assembly.anglePitch(160)
                ))
                // To drop 3rd sample into high basket
                .strafeToLinearHeading(new Vector2d(11, 35), Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
                        new SleepAction(1),
                        assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                        new SleepAction(1),
                        assembly.flipMid(),
                        new SleepAction(1.5),
                        assembly.clawOpen(),
                        new SleepAction(1),
                        assembly.flipDown(),
                        new SleepAction(1),
                        assembly.extendSlide(450),
                        new SleepAction(2),
                        assembly.anglePitch(Assembly.PitchPosition.DOWN),
                        new SleepAction(3)

                ))

                .build();
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                assembly.anglePitch(Assembly.PitchPosition.DOWN),
//                                assembly.extendSlide(Assembly.SlidesPosition.DOWN),
                                assembly.clawClose(),
                                assembly.flipCus(0.98)
                        ),
                        telemetryPacket -> {
                            assembly.update();
                            drive.updatePoseEstimate();
                            return opModeInInit();
                        }
                )
        );


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