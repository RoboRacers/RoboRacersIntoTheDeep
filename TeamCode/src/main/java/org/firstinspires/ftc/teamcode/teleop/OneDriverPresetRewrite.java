package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.PIDController;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "LM2 Preset New ", group = "Test")
public class OneDriverPresetRewrite extends LinearOpMode {
    //Pitch Stuff
    public DcMotorImplEx pitchMotor;
    public DcMotorImplEx slidesMotor;
    ElapsedTime elapsedTime;
    Servo uno;
    Servo dos;

    Servo rotateClaw;

    Servo claw;

    public static double kG = 0.027;
//    public static double kG2 = 0.0007;
public static double kG2 = 0.003;
    public static double kP = 0.005;
    public static  double kI = 0;
    public static  double kD = 0.00045;//pitch constant
    public static double kP2 = 0.005;
    public static double kI2 = 0; // slides constant
    public static double kD2 = 0.0008;
    public static double ticksPerRightAngle = 930;
    public static double ticksPerMaxExtend = 1936;
    public static double target = 0; //slides
    public static double target2 = 100; // flip
    public static double target2Last = 100; // flip

    public static double offset = 40;
    public static double rotatePos = 0;

    PIDController slidesControl;
    PIDController pitchControl;
    //CRServo rotate;

    public static double flipPos = 0;


    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");

//        slidesMotor.setDirection(DcMotorImplEx.Direction.REVERSE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        pitchControl = new PIDController(kP, kI, kD);
        slidesControl = new PIDController(kP2, kI2, kD2);

        final double ticksToDegrees = (double) 90 /ticksPerRightAngle;
        final double ticksToInches = (double) 26 /ticksPerMaxExtend;
        elapsedTime = new ElapsedTime();

//        pitchControl.setErrorTolerance(18);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        uno = hardwareMap.get(Servo.class, "flipLeft");
        dos = hardwareMap.get(Servo.class, "flipRight");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");
        claw = hardwareMap.get(Servo.class, "claw");

        pitchMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // rotate = hardwareMap.get(CRServo.class, "rotateClaw");

        //pitchMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        //pitchMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
        //slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeInInit()) {
            flipPos = 0.8;
            pitchControl.setCoefficients(kP, kI, kD);
            uno.setPosition(flipPos);
            dos.setPosition(flipPos * 0.94);
//            target2 = 300;


            pitchControl.setSetpoint(target2);


            double feedforward2 = kG2 * ((slidesMotor.getCurrentPosition()) * ticksToInches) + 0;
            double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
            pitchMotor.setPower(feedforward3 + pid + feedforward2);


        }
        waitForStart();

        while (!isStopRequested()) {

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();
            pitchControl.setCoefficients(kP, kI, kD);
            slidesControl.setCoefficients(kP2, kI2, kD2);
            elapsedTime.startTime();

            if (gamepad1.triangle) { //y
                target2Last = target2;


                target2 = 1050;
                    //target = 1700;
                    flipPos = 0.575;
            } else if (gamepad1.cross) { // a
                ElapsedTime timer = new ElapsedTime();
                target2Last = target2;
                target2 = 270;
                //target= 1100;

                flipPos = 0.14; // Down so that we can go into middle thing
            }
            else if (gamepad1.circle) { // b

                target2 = 250;  // Pick up with claw down
                //retract slides setting pitch 90 extending slides to basket
                //                wait(1000);

                flipPos = 0.14;
            }
            else if (gamepad1.square) { // x
                //                wait(1000);
                target2 = 1150;

                flipPos = 0.75;
            }

            if(gamepad1.dpad_down){
                target= 1700;//extend
            } else if (gamepad1.dpad_left) {
                target = 1100;
            } else if (gamepad1.dpad_right) {
                target = 200;
            } else if (gamepad1.dpad_down) {
                target = 50;
            } else if (gamepad1.right_trigger>0.1) {
                target2-=20;
            }
            else if (gamepad1.left_trigger>0.1) {
                target2+=25;
            }
//            if(gamepad1.left_trigger>0.1){
//                target2= 160;//extend
//            }

            pitchControl.setSetpoint(target2);

            slidesControl.setSetpoint(target);
            double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double feedforward2 = kG2 * (slidesMotor.getCurrentPosition() * ticksToInches) + 0;
            double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
            double pid2 = slidesControl.calculate(slidesMotor.getCurrentPosition());

            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                pitchMotor.setPower(gamepad2.right_stick_y * 0.5);
            } else {
                pitchMotor.setPower(feedforward3 + pid + feedforward2);
            }
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                slidesMotor.setPower(gamepad2.left_stick_y * 0.5);
            } else {
                slidesMotor.setPower(-(pid2 + feedforward));
            }
            if (gamepad2.cross && gamepad2.right_stick_button){
                pitchMotor.setPower(0);
                slidesMotor.setPower(0);
                pitchMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
                pitchMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
                slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            //ALWAYS MULTIPLY THE RIGHT FLIP OR DOS BY 0.95 TO MAKE IT SYNC WITH THE LEFT DEPOSIT OR UNO

            uno.setPosition(flipPos);
            dos.setPosition(flipPos * 0.94);


            if (gamepad2.dpad_up){
                rotateClaw.setPosition(0.05);
            } else if (gamepad2.dpad_down) {
                rotateClaw.setPosition(0.6);
            } else if (gamepad2.dpad_left) {
                rotateClaw.setPosition(0.35);
            } else if (gamepad2.dpad_right) {
                rotateClaw.setPosition(0.96);
            }

            if (gamepad2.right_bumper){
                claw.setPosition(0.45); //close
            }else if(gamepad2.left_bumper){
                claw.setPosition(0.1); //open
            }


            if (gamepad2.dpad_right){
                rotatePos-=0.05;

            }
            else if (gamepad2.dpad_left) {
                rotatePos+=0.05;
            }

            rotateClaw.setPosition(rotatePos);


            telemetry.addData("Slides Power", slidesMotor.getPower());
            telemetry.addData("slides Pos", slidesMotor.getCurrentPosition());
            telemetry.addData("Target value Pitch", target);
            telemetry.addData("Feedforward", feedforward);
            telemetry.addData("PID Values", pid);
            telemetry.addData("Pitch Motor Position", pitchMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", pitchMotor.getPower());
            telemetry.addData("Pitch Motor Angle", (pitchMotor.getCurrentPosition() - 20 ) * ticksToDegrees);
            telemetry.addData("Uno pos", uno.getPosition());
            telemetry.addData("Dos pos", dos.getPosition());
            telemetry.update();

        }
    }
}
