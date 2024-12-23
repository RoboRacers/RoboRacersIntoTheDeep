package org.firstinspires.ftc.teamcode.teleop.test;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.PIDController;

@Config
@TeleOp(name = "Slides PID Test", group = "Test")
public class SlidesPIDTest extends LinearOpMode {
    public DcMotorImplEx slidesMotor;
    public DcMotorImplEx pitchMotor;

    public static double kG = 0.027; // pitch i think
    public static double kG2 = 0.147; // slides i think
    public static double kP = 0.005;
    public static  double kI = 0;
    public static  double kD = 0.00045;
    public static double kP2 = 0.005;
    public static double kI2 = 0;
    public static double kD2 = 0.0008;
    public static double ticksPerRightAngle = 930;
    public static double ticksPerMaxExtend = 1936;
    public static double target = 100;
    public static double target2 = 300;


    PIDController slidesControl;
    PIDController pitchControl;
    public static double offset = 40;

    @Override
    public void runOpMode() throws InterruptedException {

         slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
         pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");

// Constants for conversion
        final double ticksToDegrees = (double) 90 / ticksPerRightAngle;
        final double ticksToInches = (double) 26 / ticksPerMaxExtend;
        final double inchesToMeters = 0.0254;
        final double mass = 1; // kg
        final double g = 9.8; // m/s^2
        slidesControl = new PIDController(kP2, kI2, kD2);
        pitchControl = new PIDController(kP, kI, kD);


        while (opModeInInit()) {
        }

// Initialize motors
        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStopRequested()) {
            pitchControl.setCoefficients(kP, kI, kD);
            slidesControl.setCoefficients(kP2, kI2, kD2);


            pitchControl.setSetpoint(target2);

            slidesControl.setSetpoint(target);
            // Calculate feedforward for pitch
            double pitchAngleRadians = Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees);
            double rPitch = 0.3937 + (slidesMotor.getCurrentPosition() * ticksToInches * inchesToMeters);
            double feedforwardPitch = rPitch * mass * g * Math.cos(pitchAngleRadians); // Torque due to gravity
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
            double pid2 = slidesControl.calculate(slidesMotor.getCurrentPosition());

            // Calculate feedforward for extension (assuming constant force to counteract gravity)
            double feedforwardExtend = kG2; // You can adjust kG2 based on extension force required

            // Apply feedforward to motors
            pitchMotor.setPower(feedforwardPitch + pid);
            slidesMotor.setPower(-(feedforwardExtend + pid2));

            // Telemetry data
            telemetry.addData("Pitch Motor Position", pitchMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", pitchMotor.getPower());
            telemetry.addData("Pitch Motor Angle", (pitchMotor.getCurrentPosition() - offset) * ticksToDegrees);
            telemetry.addData("Slide Motor Position", slidesMotor.getCurrentPosition());
            telemetry.addData("Slide Motor Power", slidesMotor.getPower());
            telemetry.update();
        }

    }
}
