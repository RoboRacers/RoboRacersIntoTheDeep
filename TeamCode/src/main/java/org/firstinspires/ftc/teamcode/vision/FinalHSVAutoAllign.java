package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.actions.PIDController;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;
@Config
@TeleOp(name = "HSV Button TeleOp FINAL", group = "Vision")
public class FinalHSVAutoAllign extends LinearOpMode {

    MecanumDrive drive;
    OpenCvCamera camera;
    Servo claw;
    public AnalogInput pot;
    Servo uno;
    Servo dos;
    CombinedHSVAndAnglePipeline pipeline;


    public static double flipPos = 0;


    public double xCenter;
    public static double ticksPerRightAngle = 1000;

    public static double kP = 0.042;
    public static double kI = 0.001;
    public static double kD = 0.001;
    public static double kF = 0.4;

    public static double kPup = 0.026;
    public static double kIup = 0.000;
    public static double kDup = 0.003;
    public static double kFup = 0.22;

    public static double kPdown = 0.009;
    public static double kIdown = 0.00;
    public static double kDdown = 0.00;
    //public static double kDdown = 0.000005;
    public static double kFdown = 0.1;

    public static double kIerror = 0.0018;

    public static double targetAngle = 12; // Target angle in degrees

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    public DcMotorImplEx pitchMotor;
    // PID Constants
    PIDController pitchPID;
    public static double pitchKp = 0.028;
    public static double pitchKi = 0.0001;
    public static double pitchKd = 0.0003;//pitch constant
    public static double pitchKf = 0.22;
    public static double pitchTarget = 15;
    private double pitchAngle = 0;
    public double lastPitchTarget = 0.0;
    public static double offset = 40;
    public double yCenter;
    public double xCamera;
    public double yCamera;
    public double xErrorPixels;
    public double yErrorPixels;
    public double xErrorInches;
    public double yErrorInches;
    public double length;
    public double width;
    public double height;
    public double angle;
    public int detectedObjects;
    public double output;
    private ElapsedTime timer = new ElapsedTime();

    public AnalogInput potentiometer;

    double motorPower;


    ElapsedTime runtime = new ElapsedTime();

    public List<Actions> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        runtime.reset();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//pitchMotor = new HardwareMap(DcMotor.class, "pitchMotor");
        pipeline = new CombinedHSVAndAnglePipeline();
        camera.setPipeline(pipeline);
        claw = hardwareMap.get(Servo.class, "rotateClaw");
        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
        pitchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        uno = hardwareMap.get(Servo.class, "flipLeft");
        dos = hardwareMap.get(Servo.class, "flipRight");
        potentiometer = hardwareMap.get(AnalogInput.class,"pot");

        angle = pipeline.angle;
        pitchPID = new PIDController(pitchKp, pitchKi, pitchKd );


        final double ticksToDegrees = (double) 90 /ticksPerRightAngle;


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
//        telemetry.addData("object angle: ",pipeline.angle);
//        telemetry.update();
        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeInInit()) {
//    pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////    pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            pitchAngle = mapPotentiometerToAngle(pot.getVoltage());
//            pitchPID.setErrorTolerance(2);
//            pitchPID.setSetpoint(12);
//            telemetry.addData("PITCH ANGLE", pitchAngle);
////    telemetry.update();
//
//
//            double error = pitchTarget - pitchAngle;
//            pitchPID.setCoefficients(pitchKp, pitchKi, pitchKd);
//
//            double feedforward = pitchKf * Math.cos(Math.toRadians(pitchAngle));
//            double motorPower = pitchPID.calculate(pitchAngle) + feedforward;
//            pitchMotor.setPower(-motorPower);
//
//            lastPitchTarget = pitchTarget;

            double currentVoltage = potentiometer.getVoltage();
            double currentAngle = mapPotentiometerToAngle(currentVoltage);

            // Calculate error (using angles)
            double error = targetAngle - currentAngle;

            if (targetAngle>lastTarget){
                kP = kPup;
                kD = kDup;
                kI = kIup;
                kF = kFup;
            }else if (targetAngle<lastTarget){
                kP = kPdown;
                kD = kDdown;
                kI = kIdown;
                kF= kFdown;
            }

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
            telemetry.addData("Pot Voltage", potentiometer.getVoltage());
  // Important: Update the dashboard




            flipPos = 0.1;

//    pitchMotor.setPower(feedforward3 + pid);
            uno.setPosition(flipPos);
            dos.setPosition(flipPos * 0.94);



            xCenter = pipeline.rotatedRect.center.x;
            yCenter = pipeline.rotatedRect.center.y;
//            double feedforward3 = kG2 * Math.sin(Math.toRadians((slidesMotor.getCurrentPosition()) * ticksToInches)) + 0;


            xCamera = 320;
            yCamera = 240;

            xErrorPixels = xCenter - xCamera;
            yErrorPixels = yCamera - yCenter;


            height = pipeline.rotatedRect.size.height;
            width = pipeline.rotatedRect.size.width;

            length = Math.min(height, width);
            telemetry.addData("Length in pixels", length);

            xErrorInches = xErrorPixels * (1.5 / length);
            yErrorInches = yErrorPixels * (1.5 / length);

            detectedObjects = pipeline.getDetectedObjectsCount();

            //NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
            //0.17= min 0.96 = max
//            double output = (((pipeline.angle - 0) * (0.96 - 0.17)) / (180 - 0)) + 0.17;
            output = (((pipeline.angle - 0) * (0.96 - 0.17)) / (180 - 0)) + 0.17;

            telemetry.addData("Angle", pipeline.angle);
            telemetry.addData("Detected Objects", detectedObjects);
            telemetry.addData("X Error in Inches", xErrorInches);
            telemetry.addData("Y Error in Inches", yErrorInches);
            telemetry.update();

        }

        waitForStart();



        while (!isStopRequested()) {

            if (gamepad1.cross) {

                xCenter = pipeline.rotatedRect.center.x;
                yCenter = pipeline.rotatedRect.center.y;
//            double feedforward3 = kG2 * Math.sin(Math.toRadians((slidesMotor.getCurrentPosition()) * ticksToInches)) + 0;


                xCamera = 320;
                yCamera = 240;

                xErrorPixels = xCenter - xCamera;
                yErrorPixels = yCamera - yCenter;


                height = pipeline.rotatedRect.size.height;
                width = pipeline.rotatedRect.size.width;

                length = Math.min(height, width);
                telemetry.addData("Length in pixels", length);

                xErrorInches = xErrorPixels * (1.5 / length);
                yErrorInches = yErrorPixels * (1.5 / length);

                detectedObjects = pipeline.getDetectedObjectsCount();

                //NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
                //0.17= min 0.96 = max
//            double output = (((pipeline.angle - 0) * (0.96 - 0.17)) / (180 - 0)) + 0.17;
                output = (((pipeline.angle - 90 - 0) * (0.96 - 0.17)) / (180 - 0)) + 0.17;

                claw.setPosition(output);

                telemetry.addData("Angle", pipeline.angle);
                telemetry.addData("Detected Objects", detectedObjects);
                telemetry.addData("X Error in Inches", xErrorInches);
                telemetry.addData("Y Error in Inches", yErrorInches);

                xErrorInches = (10 * xErrorInches);
                yErrorInches = (10 * yErrorInches);

                telemetry.addData("NEW X ERROR INCHES", xErrorInches);
                telemetry.addData("NEW Y ERROR INCHES", yErrorInches);



                Action trajectory = drive.actionBuilder(new Pose2d(0, 0, 0))
//                    .strafeTo(new Vector2d((yErrorInches * 3), -(xErrorInches * 3)))
//                    .strafeTo(new Vector2d((Math.pow(yErrorInches/10,2)-3.5), -Math.pow(xErrorInches/10,2)))
                        .strafeTo(new Vector2d(((yErrorInches / 10 * 1.5) - 3), (0.5 - (xErrorInches / 10 * 2))))

                        .waitSeconds(1)
//                    .strafeTo(new Vector2d((Math.sqrt(yErrorInches)), Math.sqrt(xErrorInches)))

                        .build();


                Actions.runBlocking(new ParallelAction(
                        new SequentialAction(
                                trajectory
                        ),
                        telemetryPacket -> {

                            xErrorPixels = xCenter - xCamera;
                            yErrorPixels = yCamera - yCenter;
                            height = pipeline.rotatedRect.size.height;
                            width = pipeline.rotatedRect.size.width;


//                            xErrorInches = -(12*xErrorInches);
//                            yErrorInches = (12*yErrorInches);

                            length = Math.min(height, width);
                            telemetry.addData("Length in pixels", length);


                            xErrorInches = xErrorPixels * (1.5 / length);
                            yErrorInches = yErrorPixels * (1.5 / length);

                            telemetry.addData("Angle", pipeline.angle);
                            telemetry.addData("Detected Objects", detectedObjects);
                            telemetry.addData("X Error in Inches", xErrorInches);
                            telemetry.addData("Y Error in Inches", yErrorInches);

                            xErrorInches = -Math.pow(xErrorInches, 2);
                            yErrorInches = Math.pow(yErrorInches, 2);

                            telemetry.addData("NEW X ERROR INCHES", xErrorInches);
                            telemetry.addData("NEW Y ERROR INCHES", yErrorInches);
                            telemetry.update();
                            return gamepad1.cross;
                        }
                ));


                telemetry.update();

            }
        }
    }
    private double mapPotentiometerToAngle(double potentiometerValue) {
        return ((potentiometerValue - 1.085)/ (1.88-1.085)) * (90 - 0) -0;
    }
}