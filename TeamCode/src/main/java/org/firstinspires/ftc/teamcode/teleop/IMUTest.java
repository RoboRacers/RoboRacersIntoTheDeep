package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUTest {
    private IMU imuSensor;

    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    private ElapsedTime runtime;

    public IMUTest(IMU imu) {
        this.imuSensor = imu;
        this.dashboard = dashboard;
        this.packet = new TelemetryPacket();
        this.runtime = new ElapsedTime();

        // Initialize IMU sensor
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters imuParams = new IMU.Parameters(orientationOnRobot);
        imuSensor.initialize(imuParams);
    }

    public double getHeading(){

        YawPitchRollAngles orientation = imuSensor.getRobotYawPitchRollAngles();
        double yawAngle = orientation.getYaw(AngleUnit.DEGREES);

        return yawAngle;
    }

    public void resetYaw() {
        imuSensor.resetYaw();
    }

}
