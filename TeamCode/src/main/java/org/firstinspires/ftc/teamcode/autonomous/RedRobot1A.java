package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.RobotCore;
import org.firstinspires.ftc.teamcode.util.roadrunner.trajectorysequence.TrajectorySequence;

// Localization if it shows drift, follower if it doesn't
// - Words to code by

@Config
@Autonomous(name = "RedRobot1A", group = "16481-Template")
@Disabled /* COMMENT THIS OUT WHEN YOU CREATE A NEW OPMODE */
public class RedRobot1A extends LinearOpMode{


    @Override
    public void runOpMode() {


        RobotCore robot = new RobotCore(hardwareMap);

        TrajectorySequence traj2 = robot.drive.trajectorySequenceBuilder(new Pose2d(5.71, -67.70, Math.toRadians(89.44)))
                .splineTo(new Vector2d(6.01, -37.15), Math.toRadians(89.43))
                .splineTo(new Vector2d(50.79, -28.25), Math.toRadians(2.10))
                .splineTo(new Vector2d(50.35, -61.77), Math.toRadians(-89.13))
                .setReversed(true)
                .splineTo(new Vector2d(63.25, -64.44), Math.toRadians(131.99))
                .setReversed(false)
                .splineTo(new Vector2d(50.35, -61.77), Math.toRadians(-89.13))
                .setReversed(true)
                .splineTo(new Vector2d(63.25, -64.44), Math.toRadians(131.99))
                .setReversed(false)
                .splineTo(new Vector2d(50.35, -61.77), Math.toRadians(-89.13))
                .setReversed(true)
                .splineTo(new Vector2d(63.25, -64.44), Math.toRadians(131.99))
                .setReversed(false)
                .build();



        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySequenceAsync(traj2);

    }

}
