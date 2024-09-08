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
public class BlueRobot1A extends LinearOpMode{


    @Override
    public void runOpMode() {


        RobotCore robot = new RobotCore(hardwareMap);

        TrajectorySequence traj2 = robot.drive.trajectorySequenceBuilder(new Pose2d(-17.72, 67.55, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-19.35, 28.25), Math.toRadians(267.62))
                .setReversed(false)
                .splineTo(new Vector2d(-53.17, 27.81), Math.toRadians(180.75))
                .setReversed(true)
                .splineTo(new Vector2d(-63.55, 62.21), Math.toRadians(107.97))
                .setReversed(false)
                .splineTo(new Vector2d(-53.17, 27.81), Math.toRadians(180.75))
                .setReversed(true)
                .splineTo(new Vector2d(-63.55, 62.21), Math.toRadians(107.97))
                .setReversed(false)
                .splineTo(new Vector2d(-53.17, 27.81), Math.toRadians(180.75))
                .setReversed(true)
                .splineTo(new Vector2d(-63.55, 62.21), Math.toRadians(107.97))
                .setReversed(false)
                .build();



        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySequenceAsync(traj2);

    }

}
