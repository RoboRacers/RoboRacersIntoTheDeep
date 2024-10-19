package org.firstinspires.ftc.teamcode.SubSystems.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Drivetrain.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Drivetrain.DrivetrainSubsystem;


@TeleOp(name = "Drivetrain Control LinearOpMode", group = "DrivetrainOpMode")
public class DrivetrainControlLinearOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DrivetrainSubsystem drivetrainSubsystem;
        DrivetrainCommand drivetrainCommand;
        // Initialize the subsystem
        drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap, telemetry);

        // Wait for start button to be pressed on the driver station
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        // Run until the end of the match (until stop is pressed)
        while (opModeIsActive()) {

            if (gamepad1.a) {
                drivetrainCommand = new DrivetrainCommand(drivetrainSubsystem, DrivetrainCommand.DrivetrainState.START, telemetry);
                drivetrainCommand.execute();
                sleep(500); // Keep running for 500ms (you can adjust this)
            } else if (gamepad1.b) {
                drivetrainCommand = new DrivetrainCommand(drivetrainSubsystem, DrivetrainCommand.DrivetrainState.STOP, telemetry);
                drivetrainCommand.execute();
            } else if (gamepad1.a && gamepad1.b) {
                drivetrainCommand = new DrivetrainCommand(drivetrainSubsystem, DrivetrainCommand.DrivetrainState.REVERSE, telemetry);
                drivetrainCommand.execute();
                sleep(500); // Keep running for 500ms (adjust as needed)
            } else if (gamepad1.x) {
                drivetrainCommand = new DrivetrainCommand(drivetrainSubsystem, DrivetrainCommand.DrivetrainState.LEFT, telemetry);
                drivetrainCommand.execute();
            } else if (gamepad1.y) {
                drivetrainCommand = new DrivetrainCommand(drivetrainSubsystem, DrivetrainCommand.DrivetrainState.RIGHT, telemetry);
                drivetrainCommand.execute();

                // Adding a small delay to prevent overloading the CPU
                sleep(100);
            }
        }
    }
}