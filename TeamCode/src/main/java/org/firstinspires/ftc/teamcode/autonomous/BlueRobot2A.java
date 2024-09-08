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
@Autonomous(name = "BlueRobot2A", group = "16481-Template")
@Disabled /* COMMENT THIS OUT WHEN YOU CREATE A NEW OPMODE */
public class BlueRobot2A extends LinearOpMode{


    @Override
    public void runOpMode() {


        RobotCore robot = new RobotCore(hardwareMap);

        TrajectorySequence traj2 = robot.drive.trajectorySequenceBuilder(new Pose2d(7.34, 68.74, Math.toRadians(90.00)))
                .setReversed(true)
                .lineTo(new Vector2d(0.22, 26.18))
                .setReversed(false)
                .splineTo(new Vector2d(44.86, 29.44), Math.toRadians(34.42))
                .setReversed(true)
                .splineTo(new Vector2d(60.73, 60.43), Math.toRadians(84.42))
                .setReversed(false)
                .splineTo(new Vector2d(44.86, 29.44), Math.toRadians(34.42))
                .setReversed(true)
                .splineTo(new Vector2d(60.73, 60.43), Math.toRadians(84.42))
                .setReversed(false)
                .splineTo(new Vector2d(44.86, 29.44), Math.toRadians(34.42))
                .setReversed(true)
                .splineTo(new Vector2d(60.73, 60.43), Math.toRadians(84.42))
                .setReversed(false)
                .build();




        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySequenceAsync(traj2);

    }

}
