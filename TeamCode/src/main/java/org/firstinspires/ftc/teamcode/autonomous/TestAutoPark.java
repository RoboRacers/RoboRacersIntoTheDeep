package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.RobotCore;
import org.firstinspires.ftc.teamcode.util.roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Template AutoOp with Parking Options", group = "16481-Template")
public class TestAutoPark extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotCore robot = new RobotCore(hardwareMap);

        TrajectorySequence selectedTrajectory = null;

        // Far Right
        TrajectorySequence farRight = robot.drive.trajectorySequenceBuilder(new Pose2d(60, 12, Math.toRadians(90)))
                .lineTo(new Vector2d(60, 48))  
                .build();

        // Close Right
        TrajectorySequence closeRight = robot.drive.trajectorySequenceBuilder(new Pose2d(60, -12, Math.toRadians(90)))
                .splineTo(new Vector2d(24, -36), Math.toRadians(135))  
                .splineTo(new Vector2d(0, -24), Math.toRadians(180))    
                .build();

        // Far Left
        TrajectorySequence farLeft = robot.drive.trajectorySequenceBuilder(new Pose2d(-60, -12, Math.toRadians(90)))
                .lineTo(new Vector2d(-60, -48)) 
                .build();

        // Close Left
        TrajectorySequence closeLeft = robot.drive.trajectorySequenceBuilder(new Pose2d(-60, 12, Math.toRadians(90)))
                .splineTo(new Vector2d(-24, 36), Math.toRadians(45))  
                .splineTo(new Vector2d(0, 24), Math.toRadians(0))     
                .build();

        // Wait for the start command from the driver station
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Status", "Waiting for start...");
            telemetry.update();
        }

        waitForStart();

        int selectedPath = 1; 

        switch (selectedPath) {
            case 1:
                selectedTrajectory = farRight;
                telemetry.addData("Selected Path", "Far Right");
                break;
            case 2:
                selectedTrajectory = closeRight;
                telemetry.addData("Selected Path", "Close Right");
                break;
            case 3:
                selectedTrajectory = farLeft;
                telemetry.addData("Selected Path", "Far Left");
                break;
            case 4:
                selectedTrajectory = closeLeft;
                telemetry.addData("Selected Path", "Close Left");
                break;
            default:
                telemetry.addData("Error", "No path selected.");
                break;
        }
        telemetry.update();

        // Execute the selected trajectory
        if (selectedTrajectory != null && opModeIsActive()) {
            robot.drive.followTrajectorySequenceAsync(selectedTrajectory);

            // Update the robot drive continuously to follow the trajectory
            while (opModeIsActive() && !isStopRequested()) {
                robot.drive.update();

                // Optionally, add telemetry for current position feedback
                Pose2d currentPose = robot.drive.getPoseEstimate();
                telemetry.addData("Current Position", currentPose);
                telemetry.update();
            }
        }
    }
}
