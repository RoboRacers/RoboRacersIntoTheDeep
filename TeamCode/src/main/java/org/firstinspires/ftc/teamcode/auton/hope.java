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


@Autonomous(name = "Bet hopefully", group = "16481-IntoTheDeep")
public class hope extends LinearOpMode {

    Assembly assembly;


    MecanumDrive drive;


    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        assembly = new Assembly(hardwareMap);

        runtime.reset();

        Action traj = drive.actionBuilder(new Pose2d(0,0,0))
                // Init Position
                .stopAndAdd(new SequentialAction(
                        assembly.clawClose(),
                        assembly.flipDown(),
                        assembly.anglePitch(Assembly.PitchPosition.DOWN),
//                        new SleepAction(2),
                        assembly.extendSlide(Assembly.SlidesPosition.DOWN)

//                        new SleepAction(2)
                ))
                .splineToLinearHeading(new Pose2d(7.5,36,Math.toRadians(-45)),Math.toRadians(-45))

//           above or this one     .splineToLinearHeading(new Pose2d(11,34,Math.toRadians(0)),Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        // To score Preload in first basket
                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
                        new SleepAction(1),
                        assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                        new SleepAction(0.5),
                        assembly.flipMid(),
                        new SleepAction(0.75),
                        assembly.clawOpen(),
                        new SleepAction(0.5),
                        assembly.flipDown(),
                        assembly.anglePitch(800),
                        assembly.extendSlide(450),
                        new SleepAction(1),
                        assembly.anglePitch(Assembly.PitchPosition.DOWN)
                ))
                // To pick up 1st sample(right side) on floor
                .strafeToLinearHeading(new Vector2d(23.5, 34), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.7),
                        assembly.anglePitch(110),
                        new SleepAction(0.7),
                        assembly.clawClose(),
                        new SleepAction(0.7),
                        assembly.anglePitch(250),
                        assembly.extendSlide(400)
                        ))
                // To drop 1st sample from floor into high basket
                .strafeToLinearHeading(new Vector2d(6.5, 36), Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
                        new SleepAction(1),
                        assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                        new SleepAction(0.5),
                        assembly.flipMid(),
                        new SleepAction(0.5),
                        assembly.clawOpen(),
                        new SleepAction(0.5),
                        assembly.flipDown(),
                        assembly.anglePitch(800),
                        assembly.extendSlide(450),
                        new SleepAction(0.5),
                        assembly.anglePitch(Assembly.PitchPosition.DOWN)
                ))
                // To pickup 2nd sample(middle) from the floor
                .strafeToLinearHeading(new Vector2d(21.5, 40.5), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.7),
                        assembly.anglePitch(110),
                        new SleepAction(0.7),
                        assembly.clawClose(),
                        new SleepAction(0.7),
                        assembly.extendSlide(400)
                ))
                // To drop 2nd sample into high basket
                .strafeToLinearHeading(new Vector2d(6, 37), Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        assembly.extendSlide(Assembly.SlidesPosition.DOWN),
                        new SleepAction(0.5),
                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
                        new SleepAction(1),
                        assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                        new SleepAction(1),
                        assembly.flipMid(),
                        new SleepAction(1),
                        assembly.clawOpen(),
                        new SleepAction(0.5),
                        assembly.flipDown(),
                        assembly.extendSlide(500)
                        ))
                // To pickup 3rd sample(left) from the floor
                .strafeToLinearHeading(new Vector2d(34.5, 35), Math.toRadians(90))
                .stopAndAdd(new SequentialAction(
                        //assembly.extendSlide(475),
                        assembly.flipCus(0.2),
                        assembly.rotateClaw(0.87),
                        assembly.anglePitch(110),
                        assembly.clawOpen(),
                        new SleepAction(0.5),
                        assembly.clawClose(),
                        new SleepAction(0.7),
                        assembly.anglePitch(Assembly.PitchPosition.DOWN)
                ))
                // To drop 3rd sample into high basket
                .strafeToLinearHeading(new Vector2d(8, 35), Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        assembly.anglePitch(Assembly.PitchPosition.HIGH),
                        new SleepAction(0.5),
                        assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                        new SleepAction(0.5),
                        assembly.flipMid(),
                        new SleepAction(0.5),
                        assembly.clawOpen(),
                        new SleepAction(0.5),
                        assembly.extendSlide(0),
                        assembly.flipDown(),
                        new SleepAction(0.5),
//                        assembly.extendSlide(450)
//                        new SleepAction(2),
                        assembly.flipUp(),
                        assembly.anglePitch(0)

//                        new SleepAction(3)

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