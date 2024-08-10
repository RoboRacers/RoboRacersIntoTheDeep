package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name="IMU Test", group="Linear Opmode")
public class ImuTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imuSensor;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    WeightedMovingAverageEdited RotationZ;

    @Override
    public void runOpMode() {
        imuSensor = hardwareMap.get(IMU.class, "imu");

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        RotationZ = new WeightedMovingAverageEdited(0.7);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imuSensor.initialize(new IMU.Parameters(orientationOnRobot));


        waitForStart();
        imuSensor.resetYaw();
        double yawAngle;
        double movingAvg;

        while(opModeIsActive()){
            YawPitchRollAngles orientation = imuSensor.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imuSensor.getRobotAngularVelocity(AngleUnit.DEGREES);

            yawAngle = orientation.getYaw(AngleUnit.DEGREES);
            movingAvg = RotationZ.getAvg(yawAngle);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", yawAngle);
            telemetry.addData("Yaw (Z) Filtered", "%.2f Deg. (Heading)", movingAvg);
            //telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            //telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            //telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();

            packet.put("Yaw (Z)", yawAngle);
            packet.put("Yaw (Z) Filtered", movingAvg);
            //packet.put("Yaw (Z) velocity", angularVelocity.zRotationRate);
            //packet.put("Pitch (X) velocity", angularVelocity.xRotationRate);
            //packet.put("Roll (Y) velocity", angularVelocity.yRotationRate);
            dashboard.sendTelemetryPacket(packet);

        }
    }
}