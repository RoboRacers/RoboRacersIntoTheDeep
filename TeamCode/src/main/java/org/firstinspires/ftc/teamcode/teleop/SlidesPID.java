package org.firstinspires.ftc.teamcode.teleop;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.PIDController;

@Config
@TeleOp(name = "A + Slides PID", group = "Test")
public class SlidesPID extends LinearOpMode {
    public DcMotorImplEx slidesMotor;
    public DcMotorImplEx pitchMotor;

    public static double kG = 0.027; // pitch i think
    public static double kG2 = 0.01; // slides i think
//    public static double kG2 = 0.147; // slides i think
    public static double kP = 0.005;
    public static  double kI = 0;
    public static  double kD = 0.00045;
    public static double kP2 = 0.0025;
    public static double kI2 = 0;
    public static double kD2 = 0.00072;
    public static double ticksPerRightAngle = 1000;
    public static double ticksPerMaxExtend = 1750;
    public static double target = 100;
    public static double target2 = 300;


    PIDController slidesControl;
    PIDController pitchControl;
    public static double offset = 40;

    @Override
    public void runOpMode() throws InterruptedException {

//        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
//        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
//        slidesMotor.setDirection(DcMotorImplEx.Direction.REVERSE);



        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");

// Constants for conversion
//        final double ticksToDegrees = (double) 90 / ticksPerRightAngle;
//        final double ticksToInches = (double) 26 / ticksPerMaxExtend;
//        final double inchesToMeters = 0.0254;
//        final double mass = 1.5; // kg
//        final double g = 9.8; // m/s^2
        slidesControl = new PIDController(kP2, kI2, kD2);
        pitchControl = new PIDController(kP, kI, kD);

        final double ticksToDegrees = (double) 90 /ticksPerRightAngle;
        final double ticksToInches = (double) 26 /ticksPerMaxExtend;

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

            double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
//            double feedforward3 = kG2 * Math.sin(Math.toRadians((slidesMotor.getCurrentPosition()) * ticksToInches)) + 0;
            double feedforward2 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + (kG2*(slidesMotor.getCurrentPosition()*ticksToInches));
//            // Calculate feedforward for pitch
//            double pitchAngleRadians = Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees);
//            double rPitch = 0.3937 + (slidesMotor.getCurrentPosition() * ticksToInches * inchesToMeters);
//            double feedforwardPitch = rPitch * mass * g * Math.cos(pitchAngleRadians); // Torque due to gravity
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
            double pid2 = slidesControl.calculate(slidesMotor.getCurrentPosition());

//            telemetry.addData("Feedforward", feedforward);
            // Calculate feedforward for extension (assuming constant force to counteract gravity)
            double feedforwardExtend = kG2; // You can adjust kG2 based on extension force required

            pitchMotor.setPower(pid + feedforward2);
            slidesMotor.setPower(-(pid2 + feedforward));
            telemetry.addData("slide power", slidesMotor.getPower());
            telemetry.addData("Pitch Motor Position", slidesMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", slidesMotor.getPower());
            telemetry.addData("Pitch Current", slidesMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Pitch Motor Angle", (slidesMotor.getCurrentPosition() - 45 ) * ticksToDegrees);
            telemetry.update();
//            // Apply feedforward to motors
//            pitchMotor.setPower(feedforwardPitch + pid);
//            slidesMotor.setPower(-(feedforwardExtend + pid2));
//
//            // Telemetry data
//            telemetry.addData("Pitch Motor Position", pitchMotor.getCurrentPosition());
//            telemetry.addData("Pitch Motor Power", pitchMotor.getPower());
//            telemetry.addData("Pitch Motor Angle", (pitchMotor.getCurrentPosition() - offset) * ticksToDegrees);
//            telemetry.addData("Slide Motor Position", slidesMotor.getCurrentPosition());
//            telemetry.addData("Slide Motor Power", slidesMotor.getPower());
//            telemetry.update();
        }

    }
}