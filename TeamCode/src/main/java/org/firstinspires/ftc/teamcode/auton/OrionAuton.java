package org.firstinspires.ftc.teamcode.auton;



import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Deposit;
import org.firstinspires.ftc.teamcode.robot.Intake;

@Autonomous(name = "Orion AutoOp", group = "Test")
public class OrionAuton extends LinearOpMode {

    MecanumDrive drive;

    ElapsedTime runtime = new ElapsedTime();
    Intake intake;
    Deposit deposit;

    ServoImplEx intakeFlipRight;
    ServoImplEx intakeFlipLeft;
    ServoImplEx intakeV4b;

    DcMotorImplEx intakeMotor;
    DcMotorImplEx slidesMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        deposit = new Deposit(hardwareMap);
        intake = new Intake(hardwareMap);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 90));
         intakeFlipRight = hardwareMap.get(ServoImplEx.class, "depositFlipRight");
         intakeFlipLeft = hardwareMap.get(ServoImplEx.class, "depositFlipLeft");
         intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
         slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");

         intakeV4b = hardwareMap.get(ServoImplEx.class, "depositV4bServo");

        slidesMotor.setDirection(DcMotorImplEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorImplEx.Direction.REVERSE);
        runtime.reset();

        Action traj1 = drive.actionBuilder(new Pose2d(0,0,90))
                .strafeToLinearHeading(new Vector2d(-10, 15), 90)
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {
                            deposit.v4bServo.setPosition(0.5);
                            deposit.flipLeft.setPosition(0.15);
                            deposit.flipRight.setPosition(0.15);
                            return false;
                        }
                        )
                )
                .stopAndAdd(new SleepAction(0.5))
                .strafeToLinearHeading(new Vector2d(-15, 30), 90)
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {
//                            deposit.extendoLeft.setPosition(0.9);
//                            deposit.extendoRight.setPosition(0.9);
                            return false;
                        })
                )
                .stopAndAdd(new SleepAction(1))
                .strafeToLinearHeading(new Vector2d(-15, 30), 90)
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {
                            deposit.v4bServo.setPosition(0.35);

                            deposit.flipLeft.setPosition(0.19);
                            deposit.flipRight.setPosition(0.19);


                            deposit.claw.setPosition(0.82);
                            return false;
                        })
                )
                .stopAndAdd(new SleepAction(1))
                .strafeToLinearHeading(new Vector2d(-15, 10), 90)
                .build();

        
        Action traj2 = drive.actionBuilder(new Pose2d(30,-5,0))
                .strafeToLinearHeading(new Vector2d(25, 45), 0)
                .stopAndAdd(new SleepAction(1))
                .strafeToLinearHeading(new Vector2d(25, 50), 0)
                .stopAndAdd(new SleepAction(1))
                .strafeToLinearHeading(new Vector2d(30, 10), 0)
                .build();


        waitForStart();

        while (opModeInInit()){
            deposit.claw.setPosition(0.95);
            deposit.v4bServo.setPosition(0.03);
            deposit.flipLeft.setPosition(0.65);
            deposit.flipRight.setPosition(0.65);
            intakeFlipLeft.setPosition(0.96-0.01);
            intakeV4b.setPosition(0.18);
            intakeFlipRight.setPosition(0.96);
        }

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        traj1
                ),
                telemetryPacket -> {
                    // telemtry here
                    telemetry.update();
                    return opModeIsActive();
                }
        ));
    }
}
