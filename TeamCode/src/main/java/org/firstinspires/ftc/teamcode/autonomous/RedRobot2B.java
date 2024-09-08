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
@Autonomous(name = "RedRobot2B", group = "16481-Template")
@Disabled /* COMMENT THIS OUT WHEN YOU CREATE A NEW OPMODE */
public class RedRobot2B extends LinearOpMode{


    @Override
    public void runOpMode() {


        RobotCore robot = new RobotCore(hardwareMap);

        TrajectorySequence traj2 = robot.drive.trajectorySequenceBuilder(new Pose2d(-20.24, -65.33, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-63.25, -65.33), Math.toRadians(243.43))
                .splineTo(new Vector2d(-52.87, -28.10), Math.toRadians(74.42))
                .splineTo(new Vector2d(-59.10, -65.47), Math.toRadians(260.54))
                .splineTo(new Vector2d(-52.87, -28.10), Math.toRadians(74.42))
                .splineTo(new Vector2d(-59.10, -65.47), Math.toRadians(260.54))
                .splineTo(new Vector2d(-52.87, -28.10), Math.toRadians(74.42))
                .splineTo(new Vector2d(-59.10, -65.47), Math.toRadians(260.54))
                .build();




        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySequenceAsync(traj2);

    }

}
