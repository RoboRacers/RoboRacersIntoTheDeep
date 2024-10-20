package org.firstinspires.ftc.teamcode.SubSystems.Outtake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Outtake.OuttakeSubsystem;

@TeleOp(name = "Outtake Control LinearOpMode", group = "LinearOpMode")
public class OuttakeLinearOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OuttakeSubsystem outtakeSubsystem;
        OuttakeCommand outtakeCommand;
        // Initialize the subsystem
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap, telemetry);

        // Wait for start button to be pressed on the driver station
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        // Run until the end of the match (until stop is pressed)
        while (opModeIsActive()) {
            // Button 'A' starts outtake, 'B' stops, and 'Y' reverses
            if (gamepad1.a) {
                outtakeCommand = new OuttakeCommand(outtakeSubsystem, OuttakeCommand.OuttakeState.START, telemetry);
                outtakeCommand.execute();
                sleep(500); // Keep running for 500ms (you can adjust this)
            } else if (gamepad1.b) {
                outtakeCommand = new OuttakeCommand(outtakeSubsystem, OuttakeCommand.OuttakeState.STOP, telemetry);
                outtakeCommand.execute();
            } else if (gamepad1.y) {
                outtakeCommand = new OuttakeCommand(outtakeSubsystem, OuttakeCommand.OuttakeState.REVERSE, telemetry);
                outtakeCommand.execute();
                sleep(500); // Keep running for 500ms (adjust as needed)
            }

            // Adding a small delay to prevent overloading the CPU
            sleep(100);
        }
    }
}
