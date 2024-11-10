package org.firstinspires.ftc.teamcode.auto;



import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Deposit;
import org.firstinspires.ftc.teamcode.teleop.PIDController;
import org.firstinspires.ftc.teamcode.teleop.Rolling;


@Autonomous(name = "LM1 Auton Score + Specimen", group = "16481-IntoTheDeep")
public class autonSpecimen extends LinearOpMode {

    MecanumDrive drive;
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

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        // MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        slideMotor = hardwareMap.get(DcMotor.class, "Horizontal_Slides");
        flipLeftIntake = hardwareMap.get(ServoImplEx.class, "Flip_Left_Intake");
        flipRightIntake = hardwareMap.get(ServoImplEx.class, "Flip_Right_Intake");
        intakeMotor = hardwareMap.get(CRServoImplEx.class, "Servo_Intake");
        flipRightDeposit = hardwareMap.get(ServoImplEx.class, "Flip_Right_Deposit");
        flipLeftDeposit = hardwareMap.get(ServoImplEx.class, "Flip_Left_Deposit");
        pitch = hardwareMap.get(ServoImplEx.class, "Pitch");
        claw = hardwareMap.get(ServoImplEx.class, "Claw");

        slidesRight = hardwareMap.get(DcMotorImplEx.class, "Slides_Right");
        slidesLeft = hardwareMap.get(DcMotorImplEx.class, "Slides_Left");

        slidesLeft.setDirection(DcMotorImplEx.Direction.REVERSE);
        slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(CRServoImplEx.Direction.REVERSE);

        runtime.reset();
        waitForStart();

        Action traj1 = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeToLinearHeading(new Vector2d(10, 36), -45)
                .build();

        Action traj2 = drive.actionBuilder(new Pose2d(12,36, -45))
                .strafeToConstantHeading(new Vector2d(9,43))
                .build();

        Action traj3 = drive.actionBuilder(new Pose2d(5, 40, 90))
                .strafeTo(new Vector2d(48, 40))
                .build();

        Action traj4 = drive.actionBuilder(new Pose2d(5, 40, 0))
                .splineToConstantHeading(new Vector2d(15, -60), 0)
                .build();
        Action traj5 = drive.actionBuilder(new Pose2d(15, -60,0))
                .splineToConstantHeading(new Vector2d(20,-15),0)
                .build();

        Actions.runBlocking(new SequentialAction(
//                (p) -> {
//                    flipLeftIntake.setPosition(0.85);
//                    flipRightIntake.setPosition(0.85);
//                    sleep(1000);
//                    //claw.setPosition(0.35);
//                    sleep(1000);
//                    flipLeftDeposit.setPosition(0.30);
//                    flipRightDeposit.setPosition(0.30);
//                    sleep(1000);
//
//
//
//                    return false;
//                },
                traj1,
//                (p) -> {
//                    slidesLeft.setTargetPosition(-1900);
//                    slidesRight.setTargetPosition(-1950);
//
//                    slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    slidesLeft.setPower(-.8);
//                    slidesRight.setPower(-.8);
//                    sleep(3000);
//                    pitch.setPosition(0);
//                    return false;
//                },
                traj2,
//                (p) -> {
//                    // open claw to drop sample in basket
//                    //claw.setPosition(0.1);
//                    sleep(1000);
//
//                    // flip deposit
//                    flipRightDeposit.setPosition(0.85);
//                    flipLeftDeposit.setPosition(0.85);
//                    // claw angle moves to pickup from the back
//                    pitch.setPosition(0.8);
//
//                    // reset slides to initial
//
//                    slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slidesLeft.setTargetPosition(0);
//                    slidesRight.setTargetPosition(30);
////
//                    slidesRight.setPower(-0.4);
//                    slidesLeft.setPower(-0.4);
//
//                    // reset claw angle by flipping it back
//                    pitch.setPosition(0.45);
//                    sleep(100);
//
//                    return slidesLeft.isBusy();
//                },
                traj4,
//                (p) -> {
//                    //
//                    flipRightDeposit.setPosition(0.80);
//                    flipLeftDeposit.setPosition(0.80);
//                    //claw.setPosition(0.35);
//                    sleep(2000);
//                    return false;
//                },
                traj5
        ));

        while (opModeIsActive()) {



            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            //  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }
    }
}
