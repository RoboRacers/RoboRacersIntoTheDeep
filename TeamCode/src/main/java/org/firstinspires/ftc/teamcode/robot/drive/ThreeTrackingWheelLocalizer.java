package org.firstinspires.ftc.teamcode.robot.drive;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

@Config
public class ThreeTrackingWheelLocalizer extends com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer{
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.84657;
    public static double GEAR_RATIO = 1;
    public static double LATERAL_DISTANCE = -8.38;
    public static double FORWARD_OFFSET = -11;
    public static double X_MULTIPLIER = 1.1176;
    public static double Y_MULTIPLIER = 1.1176;
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    public ThreeTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBack")); // Port Number 2
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront")); // Port Number 2
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront")); // Port Number 3

        rightEncoder.setDirection(Encoder.Direction.REVERSE);


    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }

}