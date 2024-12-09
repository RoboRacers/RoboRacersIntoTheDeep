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


@Autonomous(name = "Specimen Backup", group = "16481-IntoTheDeep")
public class BackupSpecimenAuton extends LinearOpMode {

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
                .splineToLinearHeading(new Pose2d(5,-50,Math.toRadians(0)),Math.toRadians(0))

//           above or this one     .splineToLinearHeading(new Pose2d(11,34,Math.toRadians(0)),Math.toRadians(-45))
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