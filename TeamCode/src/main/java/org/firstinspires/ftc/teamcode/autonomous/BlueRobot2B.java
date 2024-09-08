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
public class BlueRobot2B extends LinearOpMode{


    @Override
    public void runOpMode() {


        RobotCore robot = new RobotCore(hardwareMap);

        TrajectorySequence traj2 = robot.drive.trajectorySequenceBuilder(new Pose2d(11.94, 63.99, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(58.65, 65.03), Math.toRadians(-0.69))
                .setReversed(false)
                .splineTo(new Vector2d(51.39, 28.25), Math.toRadians(258.82))
                .splineTo(new Vector2d(61.77, 62.66), Math.toRadians(73.21))
                .splineTo(new Vector2d(51.39, 28.25), Math.toRadians(258.82))
                .splineTo(new Vector2d(61.77, 62.66), Math.toRadians(73.21))
                .splineTo(new Vector2d(51.39, 28.25), Math.toRadians(258.82))
                .splineTo(new Vector2d(61.77, 62.66), Math.toRadians(73.21))
                .build();




        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySequenceAsync(traj2);

    }

}
