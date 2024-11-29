package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.PIDController;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "LM2", group = "Test")
public class TotalDepositTestBetter extends LinearOpMode {
    //Pitch Stuff
    public DcMotorImplEx pitchMotor;
    public DcMotorImplEx slidesMotor;
    Servo uno;
    Servo dos;

    CRServo rotateClaw;

            Servo claw;

    public static double kG = 0.027;
    public static double kG2 = 0.1;
    public static double kP = 0.005;
    public static  double kI = 0;
    public static  double kD = 0.00045;
    public static double kP2 = 0.006;
    public static double kI2 = 0;
    public static double kD2 = 0.0005;
    public static double ticksPerRightAngle = 930;
    public static double ticksPerMaxExtend = 1936;
    public static double target = 0;
    public static double target2 = 300;
    public static double offset = 40;

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

//        pitchControl.setErrorTolerance(18);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        uno = hardwareMap.get(Servo.class, "flipLeft");
        dos = hardwareMap.get(Servo.class, "flipRight");
        rotateClaw = hardwareMap.get(CRServo.class, "rotateClaw");
        claw = hardwareMap.get(Servo.class, "claw");


        // rotate = hardwareMap.get(CRServo.class, "rotateClaw");
        pitchMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeInInit()) {
            flipPos = 1;
            pitchControl.setCoefficients(kP, kI, kD);
            uno.setPosition(flipPos);
            dos.setPosition(flipPos * 0.95);
            target2 = 250;


            pitchControl.setSetpoint(target2);


            double feedforward2 = kG2 * Math.sin(Math.toRadians((slidesMotor.getCurrentPosition()) * ticksToInches)) + 0;
            double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
            pitchMotor.setPower(feedforward3 + pid + feedforward2);


        }



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

            if (gamepad1.triangle) { //y
                target2 = 1010; //90deg + little more
                target=1650;
                flipPos = 0.675;
            } else if (gamepad1.cross) { // a
                target2 = 300;
                target= 400;
                flipPos = 0.525;
            } else if (gamepad1.circle) { // b
                target2 = 130;
            }else if (gamepad1.square) { // x
                target2 = 500;
            }

//            if(gamepad2.right_trigger>0.1){
//                slidesMotor.setPower(-0.6);//extend
//            } else if (gamepad2.left_trigger>0.1) {
//                slidesMotor.setPower(0.8); //retract
//            }else{
//                slidesMotor.setPower(0);
//            }

            if (gamepad2.square){
                target = 350;
            }else if (gamepad2.cross){
                target = 1650;
            }

            pitchControl.setSetpoint(target2);

            slidesControl.setSetpoint(target);
            double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double feedforward2 = kG2 * Math.sin(Math.toRadians((slidesMotor.getCurrentPosition()) * ticksToInches)) + 0;
            double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
            double pid2 = slidesControl.calculate(slidesMotor.getCurrentPosition());


            pitchMotor.setPower(feedforward3 + pid + feedforward2);
            slidesMotor.setPower(-(pid2 + feedforward));



            //ALWAYS MULTIPLY THE RIGHT FLIP OR DOS BY 0.95 TO MAKE IT SYNC WITH THE LEFT DEPOSIT OR UNO
            if(gamepad2.dpad_up){
                flipPos = 0.8;  //flip to score
            }else if(gamepad2.dpad_down){
                flipPos = 0.25;//flip to pick
            } else if (gamepad2.dpad_right) {
                flipPos=0.525;
            } else if (gamepad2.dpad_left) {
                flipPos = 0.69;

            }


            uno.setPosition(flipPos);
            dos.setPosition(flipPos * 0.95);

            if (gamepad2.right_bumper){
                claw.setPosition(0.785); //close
            }else if(gamepad2.left_bumper){
                claw.setPosition(0.5); //open
            }

            if(gamepad2.triangle){
                rotateClaw.setPower(-0.2);
            }else if(gamepad2.circle){
                rotateClaw.setPower(0.2);
            }else {
                rotateClaw.setPower(0);
            }




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
