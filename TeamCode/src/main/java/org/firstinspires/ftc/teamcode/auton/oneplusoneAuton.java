package org.firstinspires.ftc.teamcode.auton;



import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Assembly;


@Autonomous(name = "1+1 Auton", group = "Test")
public class oneplusoneAuton extends LinearOpMode {

    MecanumDrive drive;
    Assembly assembly;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        while (!isStopRequested()){
        telemetry.addData("runnung", "running");
        }

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        assembly = new Assembly(hardwareMap);
        runtime.reset();
        waitForStart();

        Action traj1 = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeToLinearHeading(new Vector2d(10, 40), 325)
                .build();

        Action traj2 = drive.actionBuilder(new Pose2d(10,40, 320))
                .strafeToLinearHeading(new Vector2d(23,31),0)
                .build();

        Action traj3 = drive.actionBuilder(new Pose2d(23, 31, 0))
                .strafeToLinearHeading(new Vector2d(10, 40), 325)
                .build();

//        Action traj4 = drive.actionBuilder(new Pose2d(23, 31, 0))
//                .strafeToLinearHeading(new Vector2d(10, 40), 325)
//                .build();




        Actions.runBlocking(new SequentialAction(
                assembly.clawClose(),
                assembly.extendSlide(Assembly.SlidesPosition.DOWN),
                assembly.anglePitch(Assembly.PitchPosition.DOWN),
                traj1,
                assembly.anglePitch(Assembly.PitchPosition.HIGH),
                new SleepAction(1000),
                assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                new SleepAction(500),
                assembly.flipUp(),
                new SleepAction(1000),
                assembly.clawOpen(),
                new SleepAction(1000),
                assembly.flipDown(),
                assembly.extendSlide(Assembly.SlidesPosition.DOWN),
                new SleepAction(1000),
                assembly.anglePitch(Assembly.PitchPosition.DOWN),
                traj2,
                assembly.flipDown(),
                new SleepAction(1000),
                assembly.clawClose(),
                traj3,
                assembly.anglePitch(Assembly.PitchPosition.HIGH),
                new SleepAction(1000),
                assembly.extendSlide(Assembly.SlidesPosition.HIGH),
                new SleepAction(1000),
                assembly.flipUp(),
                new SleepAction(1000),
                assembly.clawOpen(),
                new SleepAction(1000)
        ));
        while (opModeIsActive()) {



            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            //  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }
    }
}
