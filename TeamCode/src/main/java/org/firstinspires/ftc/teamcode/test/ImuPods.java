package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotCore;
import org.firstinspires.ftc.teamcode.util.roadrunner.util.Encoder;

@TeleOp(name="IMUS AND PODS FUSSIONSNS", group="Linear Opmode")
public class ImuPods extends LinearOpMode {
    WeightedMovingAverageEdited movingAverage;
    double movingAverageValue;
    double odoYaw;
    double newAccurateHeading;

    private ElapsedTime timer = new ElapsedTime();
    private IMU imuSensor;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder frontEncoder;

    private RobotCore robot;

    @Override
    public void runOpMode(){
        imuSensor = hardwareMap.get(IMU.class, "imu");
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        movingAverage = new WeightedMovingAverageEdited(0.7);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imuSensor.initialize(new IMU.Parameters(orientationOnRobot));

        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBack"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        robot = new RobotCore(hardwareMap);

        waitForStart();
        imuSensor.resetYaw();
        robot.telemetrySelfCheck(telemetry);



        while(opModeIsActive()){
            double yawAngle = imuSensor.getRobotYawPitchRollAngles().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
            movingAverageValue = movingAverage.getAvg(yawAngle);
            odoYaw = robot.drive.getPoseEstimate().getHeading(); //kinda impossible to test until we change coeffients in robot core
            newAccurateHeading = fuseIMUandOdometry(movingAverageValue, odoYaw, 0.7, 0.3); //change weights here and need to tune to figure out correct weights

            telemetry.addData("Yaw (Z) Moving Average", "%.2f Deg. (Heading)", movingAverageValue);
            telemetry.addData("Front Encoder", "%.2f", (double)frontEncoder.getCurrentPosition());
            telemetry.addData("RightEncoder", "%.2f", (double)rightEncoder.getCurrentPosition());
            telemetry.addData("Left Encoder", "%.2f", (double)leftEncoder.getCurrentPosition());
            telemetry.addData("Odo Value", "%.2f", odoYaw);
            telemetry.addData("New Accurate Heading", "%.2f", newAccurateHeading);
            telemetry.update();

            packet.put("Yaw (Z) Moving Average", movingAverageValue);
            packet.put("Front Encoder", frontEncoder.getCurrentPosition());
            packet.put("Left Encoder", leftEncoder.getCurrentPosition());
            packet.put("Right Encoder", rightEncoder.getCurrentPosition());
            packet.put("Odo Value", odoYaw);
            packet.put("New Accurate Heading", newAccurateHeading);
            dashboard.sendTelemetryPacket(packet);
        }
    }

    private double fuseIMUandOdometry(double imuYaw, double odoYaw, double imuWeight, double odoWeight){
        return imuWeight * imuYaw + odoWeight * odoYaw;
    }
}