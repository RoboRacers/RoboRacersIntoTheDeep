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
public class RedRobot1B extends LinearOpMode{


    @Override
    public void runOpMode() {


        RobotCore robot = new RobotCore(hardwareMap);

        TrajectorySequence traj2 = robot.drive.trajectorySequenceBuilder(new Pose2d(15.20, -67.25, Math.toRadians(90.00)))
                .splineTo(new Vector2d(15.79, -29.88), Math.toRadians(90.00))
                .splineTo(new Vector2d(51.83, -27.51), Math.toRadians(90.00))
                .splineTo(new Vector2d(52.57, -51.09), Math.toRadians(90.00))
                .splineTo(new Vector2d(-60.14, -62.36), Math.toRadians(179.92))
                .splineTo(new Vector2d(52.57, -51.09), Math.toRadians(90.00))
                .splineTo(new Vector2d(-60.14, -62.36), Math.toRadians(179.92))
                .splineTo(new Vector2d(52.57, -51.09), Math.toRadians(90.00))
                .splineTo(new Vector2d(-60.14, -62.36), Math.toRadians(179.92))
                .build();

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySequenceAsync(traj2);

    }

}
