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
@Autonomous(name = "RedRobot2A", group = "16481-Template")
@Disabled /* COMMENT THIS OUT WHEN YOU CREATE A NEW OPMODE */
public class RedRobot2A extends LinearOpMode{


    @Override
    public void runOpMode() {


        RobotCore robot = new RobotCore(hardwareMap);

        TrajectorySequence traj2 = robot.drive.trajectorySequenceBuilder(new Pose2d(-9.57, -66.36, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-6.60, -27.06))
                .splineTo(new Vector2d(-22.62, -33.59), Math.toRadians(205.73))
                .splineTo(new Vector2d(-36.41, -33.59), Math.toRadians(197.44))
                .splineTo(new Vector2d(-52.42, -28.99), Math.toRadians(184.59))
                .splineTo(new Vector2d(-61.03, -62.36), Math.toRadians(219.81))
                .splineTo(new Vector2d(-52.42, -28.99), Math.toRadians(184.59))
                .splineTo(new Vector2d(-61.03, -62.36), Math.toRadians(219.81))
                .splineTo(new Vector2d(-52.42, -28.99), Math.toRadians(184.59))
                .splineTo(new Vector2d(-61.03, -62.36), Math.toRadians(219.81))
                .splineTo(new Vector2d(-1.26, -26.32), Math.toRadians(90.00))
                .build();


        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySequenceAsync(traj2);

    }

}
