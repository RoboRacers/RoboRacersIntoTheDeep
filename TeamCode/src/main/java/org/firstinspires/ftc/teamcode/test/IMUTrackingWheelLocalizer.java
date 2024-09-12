package org.firstinspires.ftc.teamcode.test;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Objects;


public class IMUTrackingWheelLocalizer implements Localizer {
    ThreeTrackingWheelLocalizer trackingWheelLocalizer;
    IMU imu;
    Pose2d poseEstimate;
    //basic kalman filter (minimal prediction)
    kalmanfilter imuFilter;
    kalmanfilter localizerFilter;

    //advanced kalman filter (with prediction)
    //KalmanFilterHeading imuFilter2;
    //KalmanFilterHeading localizerFilter2;

    public IMUTrackingWheelLocalizer(HardwareMap hardwareMap){
        this.trackingWheelLocalizer = new org.firstinspires.ftc.teamcode.robot.drive.ThreeTrackingWheelLocalizer(hardwareMap);

        this.imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        imuFilter = new kalmanfilter(0.01, 0.01, 0.0);
        localizerFilter = new kalmanfilter(0.01, 0.01, 0.0);

        //imuFilter2 = new KalmanFilterHeading(0.01, 0.01, 0.0, 0.01);
        //localizerFilter2 = new KalmanFilterHeading(0.01, 0.01, 0.0, 0.01);
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        poseEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        //basic kalman filter update
        imuFilter.predict();
        imuFilter.update(imu.getRobotYawPitchRollAngles().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
        localizerFilter.predict();
        localizerFilter.update(trackingWheelLocalizer.getPoseEstimate().getHeading());

        //angular velocity + more advanced kalman filter update
        //imuFilter2.setAngularVelocity(imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
        //imuFilter2.predict();
        //imuFilter2.update(imu.getRobotYawPitchRollAngles().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
        //localizerFilter2.setAngularVelocity(trackingWheelLocalizer.getPoseVelocity().getHeading());
        //localizerFilter2.predict();
        //localizerFilter2.update(trackingWheelLocalizer.getPoseEstimate().getHeading());



        poseEstimate = trackingWheelLocalizer.getPoseEstimate();
        //basic kalman filter update
        poseEstimate = new Pose2d(poseEstimate.getX(),
                poseEstimate.getY(),
                ((imuFilter.getEstimate()) + localizerFilter.getEstimate())/2
        );
        //advanced kalman filter update
        //poseEstimate = new Pose2d(poseEstimate.getX(),
        //        poseEstimate.getY(),
        //        (imuWeight * imuFilter2.getEstimate()) + ((1-imuWeight) * localizerFilter2.getEstimate())
        //);
    }
}
