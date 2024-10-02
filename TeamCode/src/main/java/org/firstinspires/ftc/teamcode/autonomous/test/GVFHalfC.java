package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboracers.topgear.geometry.Vector2d;
import com.roboracers.topgear.planner.CubicBezierCurve;
import com.roboracers.topgear.planner.ParametricPath;

import org.firstinspires.ftc.teamcode.robot.customdrive.GVFMecanumDrive;


// Localization if it shows drift, follower if it doesn't
// - Words to code by

@Config
@Autonomous(name = "GVF half c Test", group = "16481")
public class GVFHalfC extends LinearOpMode{


    @Override
    public void runOpMode() {

        GVFMecanumDrive drive = new GVFMecanumDrive(hardwareMap);

        ParametricPath path1 = new CubicBezierCurve(new Vector2d(0,0), new Vector2d(18,0), new Vector2d(36,18), new Vector2d(36,36));

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        drive.setPath(path1);
        drive.setFollowing(true);
        while (opModeIsActive()) {
            drive.update();
            telemetry.addData("Following", drive.isFollowing);
            telemetry.update();
        }
    }

}