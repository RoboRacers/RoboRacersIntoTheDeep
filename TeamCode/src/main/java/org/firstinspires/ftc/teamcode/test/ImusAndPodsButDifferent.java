package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.roadrunner.util.Encoder;

@TeleOp(name="imus and odo pods fusionz", group="Linear Opmode")
public class ImusAndPodsButDifferent extends LinearOpMode {
    private DualWeightedMovingAverage averageCalculator = new DualWeightedMovingAverage(0.8, 0.2);
    private ElapsedTime timer = new ElapsedTime();
    private IMU imuSensor;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder frontEncoder;

    @Override
    public void runOpMode(){
        imuSensor = hardwareMap.get(IMU.class, "imu");
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imuSensor.initialize(new IMU.Parameters(orientationOnRobot));

        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBack"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            double imuYaw = imuSensor.getRobotYawPitchRollAngles().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
            double odoYaw = calculateOdometryHeading();

            double fusedYaw = averageCalculator.updateAverage(imuYaw, odoYaw);

            telemetry.addData("Fused Yaw", "%.2f Deg.", fusedYaw);
            telemetry.addData("IMU Yaw", "%.2f Deg.", imuYaw);
            telemetry.addData("Odometry Yaw", "%.2f Deg.", odoYaw);
            telemetry.update();

            packet.put("Fused Yaw", fusedYaw);
            packet.put("IMU Yaw", imuYaw);
            packet.put("Odometry Yaw", odoYaw);
            dashboard.sendTelemetryPacket(packet);
        }
    }

    private double calculateOdometryHeading() {
        double leftTicks = leftEncoder.getCurrentPosition();
        double rightTicks = rightEncoder.getCurrentPosition();
        // Placeholder calculation, adjust according to your robot's geometry and calibration
        double deltaTicks = rightTicks - leftTicks;
        return deltaTicks * (360.0 / 1440.0); // Assuming 1440 ticks per revolution
    }

    public class DualWeightedMovingAverage {
        private double imuWeight;
        private double odoWeight;
        private double averageYaw;

        public DualWeightedMovingAverage(double imuWeight, double odoWeight) {
            this.imuWeight = imuWeight;
            this.odoWeight = odoWeight;
            this.averageYaw = 0.0;
        }

        public double updateAverage(double imuYaw, double odoYaw) {
            averageYaw = imuYaw * imuWeight + odoYaw * odoWeight;
            return averageYaw;
        }
    }
}