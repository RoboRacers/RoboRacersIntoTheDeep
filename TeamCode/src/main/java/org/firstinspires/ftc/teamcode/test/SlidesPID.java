package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "PIDF SLIDES", group = "Arm")
public class SlidesPID extends LinearOpMode {
    private DcMotor slidesMotor;
    private AnalogInput potentiometer;
    private FtcDashboard dashboard;

    double motorPower;
    double maxPower;

    // Configuration variables (tunable via dashboard)
    public static double kP = 0.0;
    public static double kI = 0.00;
    public static double kD = 0.00;
    public static double kF = 0.0;
    public static double targetAngle = 0.0; // Target angle in degrees

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        slidesMotor = hardwareMap.get(DcMotor.class, "pitchMotor");
        potentiometer = hardwareMap.get(AnalogInput.class, "pot");


        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double currentVoltage = potentiometer.getVoltage();
            double currentAngle = mapPotentiometerToAngle(currentVoltage);

            // Calculate error (using angles)
            double error = targetAngle - currentAngle;

            integralSum += error * timer.seconds();

            double derivative = (error - lastError) / timer.seconds();

            double feedForward = kF * Math.sin(Math.toRadians(currentAngle));

            motorPower = (kP * error) + (kI * integralSum) + (kD * derivative) + feedForward;

            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

            if(Math.abs(motorPower) > Math.abs(maxPower)){
                maxPower = motorPower;
            }

            slidesMotor.setPower(motorPower);

            lastError = error;
            lastTarget = targetAngle;
            timer.reset();

            // Telemetry to dashboard
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.addData("Max Power Used", maxPower);
            telemetry.addData("Pot Voltage", potentiometer.getVoltage());
            telemetry.update();
            telemetry.update();// Important: Update the dashboard
            dashboard.getTelemetry();
        }
    }

    private double mapPotentiometerToAngle(double potentiometerValue) {

        return ((potentiometerValue - 0.47800000000000004)/ (1.1360000000000001-0.47800000000000004)) * (90 - 0) -0;
    }
}