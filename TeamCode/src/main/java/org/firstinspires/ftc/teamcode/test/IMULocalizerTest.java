package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotCore;
import org.firstinspires.ftc.teamcode.robot.drive.ThreeTrackingWheelLocalizer;


@TeleOp(name = "Imu Localizer Test", group = "16481-Template")
public class IMULocalizerTest extends LinearOpMode {

    RobotCore robot;
    IMUTrackingWheelLocalizer imuLocalizer;
    ThreeTrackingWheelLocalizer trackingWheelLocalizer;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);
        imuLocalizer = new IMUTrackingWheelLocalizer(hardwareMap);
        trackingWheelLocalizer = new ThreeTrackingWheelLocalizer(hardwareMap);


        while (opModeInInit()) {
            imuLocalizer.setPoseEstimate(new Pose2d(0, 0, 0));
        }

        while (!isStopRequested()) {
            imuLocalizer.update();
            robot.update();
            telemetry.addData("Heading", imuLocalizer.getPoseEstimate().getHeading());
            telemetry.addData("X", imuLocalizer.getPoseEstimate().getX());
            telemetry.addData("Y", imuLocalizer.getPoseEstimate().getY());
            telemetry.update();


        }
    }
}