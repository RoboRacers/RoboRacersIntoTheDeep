package org.firstinspires.ftc.teamcode.robot.customdrive;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.localization.Localizer;
import com.roboracers.pathfollower.localization.ThreeTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.util.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

@Config
public class CustomThreeTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889764;
    public static double GEAR_RATIO = 1;
    public static double LATERAL_DISTANCE = 12.557;
    public static double FORWARD_OFFSET = 4.97;
    public static double X_MULTIPLIER = 1.002255073876221;
    public static double Y_MULTIPLIER = 1.005446166720779;
    private Encoder leftEncoder, rightEncoder, frontEncoder;


    public CustomThreeTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Br")); // Port Number 2
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Bl")); // Port Number 2
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Fr")); // Port Number 3

        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);


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

    /**
     * Current robot pose velocity (optional)
     */
    @Override
    public com.roboracers.pathfollower.geometry.Pose2d getPoseVelocity() {
        return null;
    }
}