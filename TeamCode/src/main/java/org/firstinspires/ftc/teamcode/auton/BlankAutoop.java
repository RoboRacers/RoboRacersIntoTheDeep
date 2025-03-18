package org.firstinspires.ftc.teamcode.auton;



import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Disabled
@Autonomous(name = "Blank Autoop", group = "Test")
public class BlankAutoop extends LinearOpMode {

    MecanumDrive drive;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        runtime.reset();

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

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                traj1,
                traj2,
                traj3,
                traj4,
                traj5
        ));

        
        // My new comments
    }
}
