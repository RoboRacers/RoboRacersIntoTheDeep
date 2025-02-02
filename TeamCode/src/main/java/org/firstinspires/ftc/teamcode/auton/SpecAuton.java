package org.firstinspires.ftc.teamcode.auton;



import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Deposit;
import org.firstinspires.ftc.teamcode.robot.Intake;

@Disabled
@Autonomous(name = "Blank Autoop", group = "Test")
public class SpecAuton extends LinearOpMode {

    MecanumDrive drive;

    ElapsedTime runtime = new ElapsedTime();
    Intake intake;
    Deposit deposit;

    @Override
    public void runOpMode() throws InterruptedException {
        deposit = new Deposit(hardwareMap);
        intake = new Intake(hardwareMap);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        runtime.reset();

        Action traj1 = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeToLinearHeading(new Vector2d(10, -5), 0)
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {
                            deposit.advanceState();
                            deposit.advanceState();
                            return false;
                        }
                        )
                )
                .stopAndAdd(new SleepAction(1))
                .strafeToLinearHeading(new Vector2d(30, -5), 0)
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {
                            deposit.advanceState();
                            return false;
                        })
                )
                .stopAndAdd(new SleepAction(1))
                .strafeToLinearHeading(new Vector2d(45, -5), 0)
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {
                            deposit.advanceState();
                            return false;
                        })
                )
                .stopAndAdd(new SleepAction(1))
                .strafeToLinearHeading(new Vector2d(30, -5), 0)
                .build();

        
        Action traj2 = drive.actionBuilder(new Pose2d(30,-5,0))
                .strafeToLinearHeading(new Vector2d(25, 45), 0)
                .stopAndAdd(new SleepAction(1))
                .strafeToLinearHeading(new Vector2d(25, 50), 0)
                .stopAndAdd(new SleepAction(1))
                .strafeToLinearHeading(new Vector2d(30, 10), 0)
                .build();


        waitForStart();

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
