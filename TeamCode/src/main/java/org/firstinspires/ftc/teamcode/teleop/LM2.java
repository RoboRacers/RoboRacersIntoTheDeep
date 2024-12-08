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

import org.firstinspires.ftc.teamcode.modules.PIDController;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "LM2 Final", group = "Test")
public class LM2 extends LinearOpMode {
    //Pitch Stuff
    public DcMotorImplEx pitchMotor;
    public DcMotorImplEx slidesMotor;
    Servo uno;
    Servo dos;

    Servo rotateClaw;

    Servo claw;

    public static double kG = 0.027;
    public static double kG2 = 0.0007;
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

//        pitchControl.setErrorTolerance(18);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        uno = hardwareMap.get(Servo.class, "flipLeft");
        dos = hardwareMap.get(Servo.class, "flipRight");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");
        claw = hardwareMap.get(Servo.class, "claw");


        // rotate = hardwareMap.get(CRServo.class, "rotateClaw");
        pitchMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeInInit()) {
            flipPos = 0.92;
            pitchControl.setCoefficients(kP, kI, kD);
            uno.setPosition(flipPos);
            dos.setPosition(flipPos * 0.94);
            target2 = 300;


            pitchControl.setSetpoint(target2);


            double feedforward2 = kG2 * ((slidesMotor.getCurrentPosition()) * ticksToInches) + 0;
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
//                sleep(1000);
//                wait(1000);
//                target=1650;
                flipPos = 0.575;
            } else if (gamepad1.cross && target <740) { // a
                target2 = 300;
//                sleep(1000);
//                wait(1000);
//                target= 400;
                flipPos = 0.33; // Down so that we can go into middle thing
            } else if (gamepad1.circle && target <640) { // b
                flipPos = 0.13;
//                wait(1000);
//                target = 450;
                target2 = 200;  // Pick up with claw down
            }else if (gamepad1.square) { // x
                target2 = 1100;   // no function]
//                target=130;
            }

            if(gamepad1.dpad_up){
                flipPos+=0.025;
            }
            else if(gamepad1.dpad_down){
                flipPos-=0.025;
            }

            if(gamepad1.right_trigger>0.1){
                target2+= 25;
            }
            else if(gamepad1.left_trigger>0.1){
                target2-=25;
            }



//            if (gamepad2.triangle){
//                target=1650;
//            } else if (gamepad2.cross) {
//                target= 400;
//            }
            if (gamepad2.triangle){
                target = 1500;
            }
            else if (gamepad2.cross){
                target = 300;
            }

            if(gamepad2.right_trigger>0.1){
                target+= 75;//extend
            } else if (gamepad2.left_trigger>0.1) {
                target-= 75; //retract
            }

            pitchControl.setSetpoint(target2);

            slidesControl.setSetpoint(target);
            double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double feedforward2 = kG2 * (slidesMotor.getCurrentPosition() * ticksToInches) + 0;
            double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
            double pid2 = slidesControl.calculate(slidesMotor.getCurrentPosition());


            pitchMotor.setPower(feedforward3 + pid + feedforward2);
            slidesMotor.setPower(-(pid2 + feedforward));

            //ALWAYS MULTIPLY THE R   IGHT FLIP OR DOS BY 0.95 TO MAKE IT SYNC WITH THE LEFT DEPOSIT OR UNO


            uno.setPosition(flipPos);
            dos.setPosition(flipPos * 0.94);
//close left open right
            // rotate claw right dpad right
            //opposite for rother
            //top triangel slide low cross
            //manual triggers


            if (gamepad2.square) {
                rotatePos-=0.075;
            } else if (gamepad2.circle) {
                rotatePos  +=0.075;
            }


            if (gamepad2.right_bumper){
                claw.setPosition(0.42); //close
            }else if(gamepad2.left_bumper){
                claw.setPosition(0.175); //open
            }

            rotateClaw.setPosition(rotatePos*0.95);


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
