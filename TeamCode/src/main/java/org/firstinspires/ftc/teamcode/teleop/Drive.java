package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
@TeleOp(name ="DriveWORK", group = "16481-IntoTheDeep")
public class Drive extends LinearOpMode {

    public DcMotorImplEx slidesRight;
    public DcMotorImplEx slidesLeft;

    DcMotorImplEx leftFront;
    DcMotorImplEx leftBack;
    DcMotorImplEx rightFront;
    DcMotorImplEx rightBack;
    public ServoImplEx flipRightDeposit;
    public ServoImplEx flipLeftDeposit;

    public PIDController slidesPID = new PIDController(0.03, 0.01, 0.04);
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

         //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        leftFront = hardwareMap.get(DcMotorImplEx.class, "leftFront"); //Fl
        leftBack = hardwareMap.get(DcMotorImplEx.class, "leftBack");  //Bl
        rightBack = hardwareMap.get(DcMotorImplEx.class, "rightBack"); //Br
        rightFront = hardwareMap.get(DcMotorImplEx.class, "rightFront");  //Fr

        leftFront.setDirection(DcMotorImplEx.Direction.REVERSE); //first one  Fr  fixed all
        rightFront.setDirection(DcMotorImplEx.Direction.FORWARD); //Second one Fl
        leftBack.setDirection(DcMotorImplEx.Direction.FORWARD); // Third one Br
        rightBack.setDirection(DcMotorImplEx.Direction.FORWARD); //fourth one

        slidesRight = hardwareMap.get(DcMotorImplEx.class, "Slides_Right");
        slidesLeft = hardwareMap.get(DcMotorImplEx.class, "Slides_Left");

        slidesLeft.setDirection(DcMotorImplEx.Direction.REVERSE);
//
        flipRightDeposit = hardwareMap.get(ServoImplEx.class, "Flip_Right_Deposit");
        flipLeftDeposit = hardwareMap.get(ServoImplEx.class, "Flip_Left_Deposit");

        waitForStart();

        while (opModeIsActive()) {

           double drive = -gamepad1.left_stick_y;
           double strafe = gamepad1.left_stick_x;
           double rotate = gamepad1.right_stick_x;

           double frontLeftPower = drive + strafe + rotate;
           double frontRightPower = drive - strafe - rotate;
           double backLeftPower = drive - strafe + rotate;
           double backRightPower = drive + strafe - rotate;

           leftFront.setPower(frontLeftPower);
           rightFront.setPower(frontRightPower);
           leftBack.setPower(backLeftPower);
           rightBack.setPower(backRightPower);


//           if (gamepad2.square){
//               flipRightDeposit.setPosition(0.1);
//               flipLeftDeposit.setPosition(0.1);
//           } else if (gamepad2.cross) {
//               flipRightDeposit.setPosition(0.5);
//               flipLeftDeposit.setPosition(0.5);
//           } else if (gamepad2.triangle) {
//               flipRightDeposit.setPosition(0.8);
//               flipLeftDeposit.setPosition(0.8);
//           } else if (gamepad2.left_stick_y > 0.1) {
//               flipRightDeposit.setPosition(gamepad2.left_stick_y);
//               flipLeftDeposit.setPosition(gamepad2.left_stick_y);
//           }
           if (gamepad2.cross){
               //pitch.setPosition(0.275);
               slidesPID.setSetpoint(50);
               double output = slidesPID.calculate((slidesRight.getCurrentPosition() + slidesLeft.getCurrentPosition())/2);
               slidesRight.setPower(-output);
               slidesLeft.setPower(-output);telemetry.addData("Output", output);telemetry.update();
           }
           else if (gamepad2.triangle){
               //pitch.setPosition(0.28);
               slidesPID.setSetpoint(300);
               double output = slidesPID.calculate((slidesRight.getCurrentPosition() + slidesLeft.getCurrentPosition())/2);
                slidesRight.setPower(-output);
                slidesLeft.setPower(-output);
                telemetry.addData("Output", output);telemetry.update();

           }
           else if (gamepad2.square){
               //pitch.setPosition(0.28);
               slidesPID.setSetpoint(200);
               double output = slidesPID.calculate((slidesRight.getCurrentPosition() + slidesLeft.getCurrentPosition())/2);
               slidesRight.setPower(-output);
               slidesLeft.setPower(-output);telemetry.addData("Output", output);telemetry.update();
           }
           else if (gamepad2.circle){
               //pitch.setPosition(0.28);
               slidesPID.setSetpoint(100);
               double output = slidesPID.calculate((slidesRight.getCurrentPosition() + slidesLeft.getCurrentPosition())/2);
               slidesRight.setPower(-output);
               slidesLeft.setPower(-output);telemetry.addData("Output", output);telemetry.update();
           } else if (gamepad2.right_trigger>0.1) {
               slidesRight.setPower(0);
               slidesLeft.setPower(0);
           }


//            telemetry.addData("x", drive.pose.position.x);
//            telemetry.addData("y", drive.pose.position.y);

//            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));

            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("backRightPower", backRightPower);

            telemetry.addData("Right Deposit", slidesRight.getCurrentPosition());
            telemetry.addData("Left Deposit", slidesLeft.getCurrentPosition());
            telemetry.addData("Prash Baby", (slidesLeft.getCurrentPosition()+slidesRight.getCurrentPosition())/2);

            telemetry.update();



            //drive.updatePoseEstimate();
//                telemetry.addData("Flip Right value", flipRightIntake.getPosition());
//                telemetry.addData("Flip Left value", flipLeftIntake.getPosition());
//                telemetry.addData("Flip Left value", rolling.getPower());
//                telemetry.addData("ServoPos", depositRight.getPosition());
//                telemetry.addData("ServoPos", depositLeft.getPosition());
//
//                telemetry.addData("slidePosLeft", intakeMotorLeft.getCurrentPosition());
//                telemetry.addData("slidePosRight", intakeMotorRight.getCurrentPosition());




            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            //  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }

    }
}
