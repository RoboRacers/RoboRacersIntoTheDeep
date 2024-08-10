package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboracers.pathfollower.geometry.Vector2d;
import com.roboracers.pathfollower.planner.CubicBezierCurve;
import com.roboracers.pathfollower.planner.CurveBuilder;
import com.roboracers.pathfollower.planner.ParametricPath;

import org.firstinspires.ftc.teamcode.robot.RobotCore;
import org.firstinspires.ftc.teamcode.robot.customdrive.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.util.roadrunner.trajectorysequence.TrajectorySequence;


// Localization if it shows drift, follower if it doesn't
// - Words to code by

@Config
@Autonomous(name = "GVF Test", group = "16481")
public class GVFTest extends LinearOpMode{


    @Override
    public void runOpMode() {

        CustomMecanumDrive drive = new CustomMecanumDrive(hardwareMap);

        ParametricPath path1 = new CubicBezierCurve(new Vector2d(0,0), new Vector2d(36,-36), new Vector2d(36,36), new Vector2d(72,0));

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        drive.setPath(path1);
        drive.setFollowing(true);
        while (opModeIsActive()) {
            drive.update();
        }
    }

}
