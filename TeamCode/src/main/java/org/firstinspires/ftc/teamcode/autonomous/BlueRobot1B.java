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
@Autonomous(name = "BlueRobot1B", group = "16481-Template")
@Disabled /* COMMENT THIS OUT WHEN YOU CREATE A NEW OPMODE */
public class BlueRobot1B extends LinearOpMode{


    @Override
    public void runOpMode() {


        RobotCore robot = new RobotCore(hardwareMap);

        TrajectorySequence traj2 = robot.drive.trajectorySequenceBuilder(new Pose2d(-11.64, 66.07, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-10.60, 37.15), Math.toRadians(90.00))
                .splineTo(new Vector2d(-52.42, 27.51), Math.toRadians(90.00))
                .setReversed(true)
                .splineTo(new Vector2d(-52.13, 62.66), Math.toRadians(90.00))
                .splineTo(new Vector2d(30.62, 43.38), Math.toRadians(52.35))
                .setReversed(false)
                .splineTo(new Vector2d(60.58, 64.58), Math.toRadians(90.00))
                .build();



        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySequenceAsync(traj2);

    }

}
