package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;    
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.PIDController;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name ="DepositWORK", group = "16481-IntoTheDeep")
public class DepositTest extends LinearOpMode {


//    private ElapsedTime runtime = new ElapsedTime();
//
//    // PID coefficients
//    public static final double Kp = 0.01;  // Proportional coefficient
//    public static final double Ki = 0.00; // Integral coefficient
//    public static final double Kd = 0.05; // Derivative coefficient
//
//    private double integral = 0;
//    private double previousError = 0;
    public DcMotor slideMotor;
    public ServoImplEx flipLeftIntake;
    public ServoImplEx flipRightIntake;


    public CRServoImplEx intakeMotor;


    public ServoImplEx flipRightDeposit;
    public ServoImplEx flipLeftDeposit;
    public ServoImplEx pitch;
    public ServoImplEx claw;

    public DcMotorImplEx slidesRight;
    public DcMotorImplEx slidesLeft;
//    public double targetPosition = 0;

    public PIDController pidController;
    @Override
    public void runOpMode() throws InterruptedException {


           // MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        slideMotor = hardwareMap.get(DcMotor.class, "Horizontal_Slides");
        flipLeftIntake = hardwareMap.get(ServoImplEx.class, "Flip_Left_Intake");
        flipRightIntake = hardwareMap.get(ServoImplEx.class, "Flip_Right_Intake");
        intakeMotor = hardwareMap.get(CRServoImplEx.class, "Servo_Intake");


//            depositRight = hardwareMap.get(ServoImplEx.class, "")


        flipRightDeposit = hardwareMap.get(ServoImplEx.class, "Flip_Right_Deposit");
        flipLeftDeposit = hardwareMap.get(ServoImplEx.class, "Flip_Left_Deposit");
        pitch = hardwareMap.get(ServoImplEx.class, "Pitch");
        claw = hardwareMap.get(ServoImplEx.class, "Claw");

        slidesRight = hardwareMap.get(DcMotorImplEx.class, "Slides_Right");
        slidesLeft = hardwareMap.get(DcMotorImplEx.class, "Slides_Left");

        slidesLeft.setDirection(DcMotorImplEx.Direction.REVERSE);
        intakeMotor.setDirection(CRServoImplEx.Direction.REVERSE);

//        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));


                // Set your target position in encoder ticks
//                double currentPositionLeft = slidesLeft.getCurrentPosition();
//                double currentPositionRight = slidesRight.getCurrentPosition();
//                double error = targetPosition - ((currentPositionLeft + currentPositionRight)/2);
//
//                // Calculate the integral and derivative terms
//                integral += error * runtime.time();
//                double derivative = (error - previousError) / runtime.time();
//
//                // Calculate the output power
//                double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

                // Set the motor power
//                slidesLeft.setPower(-output);
//                slidesRight.setPower(-output);

                // Update the previous error
//                previousError = error;
//
//
//                // Reset the runtime for the next loop
//                runtime.reset();

                // Show telemetry
//                telemetry.addData("Target Position", targetPosition);
//                telemetry.addData("Current Position Left", currentPositionLeft);
//                telemetry.addData("Current Position Right", currentPositionRight);
//                telemetry.addData("Error", error);
//                telemetry.addData("Motor Power", output);

//                if (gamepad1.right_trigger>0.1){
//                    // extend slides
//                     slideMotor.setPower(0.4);
//                }
//                else if (gamepad1.left_trigger>0.1) {
//                    // retract slides
//                    slideMotor.setPower(-0.4);
//                }
//                else {
//                    slideMotor.setPower(0);
//                }
                if (gamepad1.right_trigger>0.1){
                    // extend slides
                    slideMotor.setPower(0.5); //0.4 & -0.4
                }
                else if (gamepad1.left_trigger>0.1) {
                    // retract slides
                    slideMotor.setPower(-0.4);
                }
                else {
                    slideMotor.setPower(0);
                    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                if (gamepad1.dpad_down) {
                    intakeMotor.setPower(-0.4);
                } else if (gamepad1.dpad_up) {
                    intakeMotor.setPower(0.7);
                } else {
                    intakeMotor.setPower(0);
                }

                if(gamepad1.left_stick_y>0.1)
                    intakeMotor.setPower(gamepad1.left_stick_y);

                if (gamepad1.cross){
                    flipLeftIntake.setPosition(0.85);
                    flipRightIntake.setPosition(0.85);
                    pitch.setPosition(0.65);
//
                } else if (gamepad1.triangle) {
                    flipLeftIntake.setPosition(0.35);
                    flipRightIntake.setPosition(0.35);
                }
                else if (gamepad1.square) {
                    flipLeftIntake.setPosition(0.3);
                    flipRightIntake.setPosition(0.3);
                }

                if(gamepad1.circle){
                    //double output = minNew + ((maxNew - minNew) / (maxOld - minOld)) * (position - minOld);
                    double output = 0.25 + ((0.95 - 0.25) / (1 - -1)) * (gamepad1.touchpad_finger_1_y - -1);
                    flipLeftIntake.setPosition(output);
                    flipRightIntake.setPosition(output);
                }

                if (gamepad2.cross){
                    //pitch.setPosition(0.1);
                    flipRightDeposit.setPosition(0.80);
                    flipLeftDeposit.setPosition(0.80);
                    pitch.setPosition(1);

                }
                else if (gamepad2.square){
                   // pitch.setPosition(0);
                    flipRightDeposit.setPosition(0.30);
                    flipLeftDeposit.setPosition(0.30);
                }
//                else if (gamepad2.circle){
//                    pitch.setPosition(0);
//                   // flipRightDeposit.setPosition(flipRightDeposit.getPosition()+0.05);
//                    //flipLeftDeposit.setPosition(flipLeftDeposit.getPosition()+0.05);
//                }
                else if (gamepad2.triangle){
                   // pitch.setPosition(0.15);
                    flipRightDeposit.setPosition(0.10);
                    flipLeftDeposit.setPosition(0.10);
                    pitch.setPosition(0);
                }




                if (gamepad2.dpad_up){
                     pitch.setPosition(0);
//                    flipRightDeposit.setPosition(0.15);
//                    flipLeftDeposit.setPosition(0.15);

                }
                else if (gamepad2.dpad_down){
                     pitch.setPosition(1);
//                    flipRightDeposit.setPosition(0.15);
//                    flipLeftDeposit.setPosition(0.15);

                }
                else if (gamepad2.dpad_right){
                     pitch.setPosition(0);
//                    flipRightDeposit.setPosition(0.15);
//                    flipLeftDeposit.setPosition(0.15);

                }
                else if (gamepad2.dpad_left){
                    pitch.setPosition(0.5);
//                    flipRightDeposit.setPosition(0.15);
//                    flipLeftDeposit.setPosition(0.15);

                }


                if(gamepad2.right_bumper && !gamepad2.circle){
                    claw.setPosition(0.05);
                }
                else if(gamepad2.left_bumper && !gamepad2.circle) {
                    claw.setPosition(0.3);
                }
//               else if (gamepad2.dpad_up) {
//                    targetPosition=15;
//                }

//                if (gamepad2.circle && gamepad2.right_bumper){
//                    if (!gamepad2.right_bumper)
//                        claw.setPosition(claw.getPosition() + 0.1);
//                }
//                if (gamepad2.circle && gamepad2.left_bumper){
//                    if (!gamepad2.right_bumper)
//                        claw.setPosition(claw.getPosition() - 0.1);
//                }

                if(gamepad2.circle)
                    claw.setPosition(gamepad2.touchpad_finger_1_x);

                if (gamepad2.left_trigger>0.1){
                    slidesLeft.setPower(0.3);
                    slidesRight.setPower(0.3);
                    claw.setPosition(0.425);
                } else if (gamepad2.right_trigger>0.1) {
                    claw.setPosition(0.425);

                    slidesLeft.setPower(-0.6);
                    slidesRight.setPower(-0.6);

                }
//                else if (gamepad2.dpad_left){
//                    slidesLeft.setPower(slidesLeft.getPower()-0.1);
//                    slidesRight.setPower(slidesRight.getPower()-0.1);
//                } else if (gamepad2.dpad_right) {
//                    slidesLeft.setPower(slidesLeft.getPower()+0.1);
//                    slidesRight.setPower(slidesRight.getPower()+0.1);
//                }
                else {
                    slidesRight.setPower(0);
                    slidesLeft.setPower(0);
                }


                // Intake
//                if (gamepad1.right_trigger > 0.5){
//                    intakeMotor.setDirection(CRServoImpl.Direction.FORWARD);
//                    intakeMotor.setPower(0.5);
//                } else if (gamepad1.left_trigger > 0.5){
//                    intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//                    intakeMotor.setPower(0.5);
//
//                }
//
//                if (gamepad1.triangle){
//                     setIntakeUp();
//                } else if (gamepad1.cross) {
//                     setIntakeDown();
//                }




                //drive.updatePoseEstimate();
//                telemetry.addData("Flip Right value", flipRightIntake.getPosition());
//                telemetry.addData("Flip Left value", flipLeftIntake.getPosition());
//                telemetry.addData("Flip Left value", rolling.getPower());
//                telemetry.addData("ServoPos", depositRight.getPosition());
//                telemetry.addData("ServoPos", depositLeft.getPosition());
//
//                telemetry.addData("slidePosLeft", intakeMotorLeft.getCurrentPosition());
//                telemetry.addData("slidePosRight", intakeMotorRight.getCurrentPosition());
                telemetry.addData("Intake Slides", slideMotor.getCurrentPosition());
                telemetry.addData("slide Right", slidesRight.getCurrentPosition());
                telemetry.addData("slide Left", slidesLeft.getCurrentPosition());

                telemetry.addData("Flip Left Deposit", flipLeftDeposit.getPosition());
                telemetry.addData("Flip Right Deposit", flipRightDeposit.getPosition());

                telemetry.addData("CLAW:", claw.getPosition());

                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
              //  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            }

    }
}
