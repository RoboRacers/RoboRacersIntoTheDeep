package org.firstinspires.ftc.teamcode.modules.pinpoint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
public final class PinpointLocalizer implements Localizer {
//    public static class Params {
//        public double par0YTicks = -1525.14753462813852; // -27.14753462813852; //  y position of the first parallel encoder (in tick units)
//        public double par1YTicks = 1275.902901802167232; //21.902901802167232; // y position of the second parallel encoder (in tick units)
//        public double perpXTicks = -2375.7267655146298; //-43.7267655146298; // x position of the perpendicular encoder (in tick units)
//    }
//
//    public static Params PARAMS = new Params();
//
//    public final Encoder par0, par1, perp;
//
//    public final double inPerTick;
//
//    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;
    private boolean initialized;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
//        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront"))); //left Encoder port 0
//        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack"))); //right encoder port 2
//        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFront"))); // back encoder port 1
//
//        // TODO: reverse encoder directions if needed
//        //   par0.setDirection(DcMotorSimple.Direction.REVERSE);
//        par0.setDirection(DcMotorSimple.Direction.REVERSE);
//        //par1.setDirection(DcMotorSimple.Direction.REVERSE);
//        //perp.setDirection(DcMotorSimple.Direction.REVERSE);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        odo.setOffsets(0, -368.3/25.4);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


//        this.inPerTick = inPerTick;
//
//        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        // Get position and velocity from your existing system
        double newXPos = odo.getPosX(); // Replace with your method to get X position
        double newYPos = odo.getPosY(); // Replace with your method to get Y position
        double newXVel = odo.getVelX(); // Replace with your method to get X velocity
        double newYVel = odo.getVelY(); // Replace with your method to get Y velocity
        double newHeadingPos = odo.getHeading(); // Replace with your method to get heading position
        double newHeadingVel = odo.getHeadingVelocity();

        // Create a new Twist2dDual object using your values
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(new DualNum<Time>(new double[] {newXPos, newYPos}),
                        new DualNum<Time>(new double[] {newXVel, newYVel})),
                new DualNum<>(new double[] {newHeadingPos, newHeadingVel}) // Set rotation to 0 if not needed
        );

        return twist;
    }
}
