package org.firstinspires.ftc.teamcode.test.kalmanfilterrssss;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.test.kalmanfilterrssss.kalmanfilter;

@TeleOp(name="IMU Test", group="Linear Opmode")
public class SensorRunning extends LinearOpMode {
    kalmanfilter KalmanFilter;
    private ElapsedTime timer = new ElapsedTime();
    private IMU imuSensor;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    @Override
    public void runOpMode(){
        imuSensor = hardwareMap.get(IMU.class, "imu");
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        KalmanFilter = new kalmanfilter(0.01, 0.01, 0);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imuSensor.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();
        imuSensor.resetYaw();

        while(opModeIsActive()){
            double yawAngle = imuSensor.getRobotYawPitchRollAngles().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
            KalmanFilter.predict();
            KalmanFilter.update(yawAngle);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", yawAngle);
            telemetry.addData("Yaw (Z) Filtered", "%.2f Deg. (Heading)", KalmanFilter.getEstimate());
            telemetry.update();

            packet.put("Yaw (Z)", yawAngle);
            packet.put("Yaw (Z) Filtered", KalmanFilter.getEstimate());


            dashboard.sendTelemetryPacket(packet);

        }
    }
}