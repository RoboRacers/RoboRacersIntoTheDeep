package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotCore;


@TeleOp(name = "Imu Localizer Test", group = "16481-Template")
public class IMULocalizerTest extends LinearOpMode {

    RobotCore robot;
    IMUTrackingWheelLocalizer imuLocalizer;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);
        imuLocalizer = new IMUTrackingWheelLocalizer(hardwareMap);

        while (opModeInInit()) {
        }

        while (!isStopRequested()) {
            telemetry.addData("Heading", imuLocalizer.getPoseEstimate().getHeading());

            imuLocalizer.update();
            robot.update();
        }
    }
}