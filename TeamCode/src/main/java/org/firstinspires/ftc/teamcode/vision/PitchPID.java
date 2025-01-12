package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.actions.PIDController;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Pitch PID", group = "Vision")
@Config
public class PitchPID extends LinearOpMode {

    MecanumDrive drive;

    Servo claw;
    public AnalogInput pot;
    Servo uno;
    Servo dos;


    public static double flipPos = 0;


    public double xCenter;
    public static double ticksPerRightAngle = 1000;

    public static double kP = 0.042;
    public static double kI = 0.001;
    public static double kD = 0.001;
    public static double kF = 0.4;

//    public static double kPup = 0.026;
//    public static double kIup = 0.000;
//    public static double kDup = 0.003;
//    public static double kFup = 0.22;
//
//    public static double kPdown = 0.009;
//    public static double kIdown = 0.00;
//    public static double kDdown = 0.00;
//    //public static double kDdown = 0.000005;
//    public static double kFdown = 0.1;

    public static double kIerror = 0.0018;

    public static double targetAngle = 12; // Target angle in degrees

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    public DcMotorImplEx pitchMotor;
    // PID Constants
    PIDController pitchPID;
//    public static double pitchKp = 0.028;
//    public static double pitchKi = 0.0001;
//    public static double pitchKd = 0.0003;//pitch constant
//    public static double pitchKf = 0.22;
//    public static double pitchTarget = 15;
//    private double pitchAngle = 0;
//    public double lastPitchTarget = 0.0;
//    public static double offset = 40;

//    public double length;
//    public double width;
//    public double height;
//    public double angle;
//    public int detectedObjects;
//    public double output;
    private ElapsedTime timer = new ElapsedTime();

    public AnalogInput potentiometer;

    double motorPower;


    ElapsedTime runtime = new ElapsedTime();

    public List<Actions> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        runtime.reset();

        claw = hardwareMap.get(Servo.class, "rotateClaw");
        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
        pitchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        uno = hardwareMap.get(Servo.class, "flipLeft");
        dos = hardwareMap.get(Servo.class, "flipRight");
        potentiometer = hardwareMap.get(AnalogInput.class,"pot");



        final double ticksToDegrees = (double) 90 /ticksPerRightAngle;


//        telemetry.addData("object angle: ",pipeline.angle);
//        telemetry.update();
        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();



        while (!isStopRequested()) {
            double currentVoltage = potentiometer.getVoltage();
            double currentAngle = mapPotentiometerToAngle(currentVoltage);

            // Calculate error (using angles)
            double error = targetAngle - currentAngle;
//
//            if (targetAngle>lastTarget){
//                kP = kPup;
//                kD = kDup;
//                kI = kIup;
//                kF = kFup;
//            }else if (targetAngle<lastTarget){
//                kP = kPdown;
//                kD = kDdown;
//                kI = kIdown;
//                kF= kFdown;
//            }

            if(Math.abs(error) < 10 && Math.abs(error)>1){
//                kP = 0.054;
//                kD = 0.0015;
                kI = kIerror;
            }


            integralSum += error * timer.seconds();


            double derivative = (error - lastError) / timer.seconds();

            double feedForward = kF * Math.cos(Math.toRadians(currentAngle));




            motorPower = (kP * error) + (kI * integralSum) + (kD * derivative) + feedForward;

            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));


            pitchMotor.setPower(motorPower);

            lastError = error;
            lastTarget = targetAngle;


            //NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
            //0.17= min 0.96 = max
//            double output = (((pipeline.angle - 0) * (0.96 - 0.17)) / (180 - 0)) + 0.17;


            telemetry.addData("Angle", currentAngle);

            telemetry.addData("Voltage", currentVoltage);
            telemetry.update();
        }
    }
    private double mapPotentiometerToAngle(double potentiometerValue) {
        return ((potentiometerValue - 0.47800000000000004)/ (1.1360000000000001-0.47800000000000004)) * (90 - 0) -0;
    }
}