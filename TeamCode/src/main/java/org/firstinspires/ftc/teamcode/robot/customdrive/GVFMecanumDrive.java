package org.firstinspires.ftc.teamcode.robot.customdrive;

import static org.firstinspires.ftc.teamcode.robot.customdrive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.robot.customdrive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.customdrive.DriveConstants.encoderTicksToInches;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.roboracers.topgear.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.roboracers.topgear.follower.GuidedVectorFieldFollower;
import com.roboracers.topgear.geometry.Pose2d;
import com.roboracers.topgear.planner.ParametricPath;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.util.roadrunner.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class GVFMecanumDrive {

    public static double LATERAL_MULTIPLIER = 1.6;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    public static final double lateralMultiplier = 1.0;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;
    private Localizer localizer;

    public GuidedVectorFieldFollower follower;



    public GVFMecanumDrive(HardwareMap hardwareMap) {

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);
        /*
        leftFront = hardwareMap.get(DcMotorEx.class, "Br"); //Fl
        leftRear = hardwareMap.get(DcMotorEx.class, "Fr"); //Bl
        rightRear = hardwareMap.get(DcMotorEx.class, "Fl"); //Br
        rightFront = hardwareMap.get(DcMotorEx.class, "Bl");// Fr

         */

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront"); //Fl
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear"); //Bl
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear"); //Br
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");// Fr

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
        localizer = new CustomThreeTrackingWheelLocalizer(hardwareMap);

        follower = new GuidedVectorFieldFollower(TuneableConstants.getParams());

    }

    private boolean isFollowing = false;

    public void update() {
        updatePoseEstimate();
        // Only adhere to the follower when path following is enabled.
        if (isFollowing) {
            // Check if the path is complete. If it is, stop following.
            if (follower.isComplete(getPoseEstimate())) {
                isFollowing = false;
            } else setDrivePower(follower.getDriveVelocity(getPoseEstimate()));
        }
    }

    /*
     * Follower functions
     */

    public void setPath (ParametricPath parametricPath) {
        follower.setPath(parametricPath);
    }

    public void setFollowing(boolean following) {
        isFollowing = following;
    }

    /**
     * Action that follows a path to completion.
     * @param path
     * @return action
     */
    public Action followPath(ParametricPath path) {
        return new Action() {
            boolean firstLoop = true;
            @Override
            public boolean run(TelemetryPacket p) {
                if (firstLoop) {
                    setPath(path);
                    setFollowing(true);
                    firstLoop = false;
                    return true;
                } else if (!follower.isComplete(getPoseEstimate())) {
                    update();
                    return true;
                }
                return false;
            }
        };
    }

    /*
     * Motor Behavior Functions
     */

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    /*
     * Drive functions
     */

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    public void setDrivePower(Pose2d drivePower) {
        List<Double> powers = robotToWheelVelocities(
                drivePower,
                1.0,
                1.0,
                lateralMultiplier
        );
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    /*
     * Wheel velocities and kinematics
     */

    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    public static List<Double> robotToWheelVelocities(Pose2d robotVel, double trackWidth, double wheelBase, double lateralMultiplier) {
        double k = (trackWidth + wheelBase) / 2.0;
        List<Double> wheelVelocities = new ArrayList<>();
        wheelVelocities.add(robotVel.getX() - lateralMultiplier * robotVel.getY() - k * robotVel.getHeading());
        wheelVelocities.add(robotVel.getX() + lateralMultiplier * robotVel.getY() - k * robotVel.getHeading());
        wheelVelocities.add(robotVel.getX() - lateralMultiplier * robotVel.getY() + k * robotVel.getHeading());
        wheelVelocities.add(robotVel.getX() + lateralMultiplier * robotVel.getY() + k * robotVel.getHeading());
        return wheelVelocities;
    }

    /**
     * Runs one iteration of the localization algorithm.
     */
    public void updatePoseEstimate() {
        this.localizer.update();
    }

    /**
     * Returns the current estimated pose of the robot.
     * @return the current pose of the robot
     */
    public Pose2d getPoseEstimate() {
        return localizer.getPoseEstimate();
    }

}
