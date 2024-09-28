package SubSystems.Intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import SubSystems.Intake.ControlIntakeCommand;
import SubSystems.Intake.IntakeSubsystem;


@TeleOp(name = "Intake Control LinearOpMode", group = "LinearOpMode")
public class IntakeControlLinearOpMode extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        IntakeSubsystem intakeSubsystem;
        ControlIntakeCommand controlIntakeCommand;
        // Initialize the subsystem
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);

        // Wait for start button to be pressed on the driver station
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        // Run until the end of the match (until stop is pressed)
        while (opModeIsActive()) {
            // Button 'A' starts intake, 'B' stops, and 'Y' reverses
            if (gamepad1.a) {
                controlIntakeCommand = new ControlIntakeCommand(intakeSubsystem, ControlIntakeCommand.IntakeState.START, telemetry);
                controlIntakeCommand.execute();
                sleep(500); // Keep running for 500ms (you can adjust this)
            } else if (gamepad1.b) {
                controlIntakeCommand = new ControlIntakeCommand(intakeSubsystem, ControlIntakeCommand.IntakeState.STOP, telemetry);
                controlIntakeCommand.execute();
            } else if (gamepad1.y) {
                controlIntakeCommand = new ControlIntakeCommand(intakeSubsystem, ControlIntakeCommand.IntakeState.REVERSE, telemetry);
                controlIntakeCommand.execute();
                sleep(500); // Keep running for 500ms (adjust as needed)
            }

            // Adding a small delay to prevent overloading the CPU
            sleep(100);
        }
    }
}
