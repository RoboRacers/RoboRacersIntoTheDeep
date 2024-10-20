package org.firstinspires.ftc.teamcode.SubSystems.Elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Elevator.ElevatorCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator.ElevatorSubsystem;


@TeleOp(name = "Elevator Control LinearOpMode", group = "ElevatorOpMode")
public class ElevatorControlLinearOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorSubsystem elevatorSubsystem;
        ElevatorCommand elevatorCommand;
        // Initialize the subsystem
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry);

        // Wait for start button to be pressed on the driver station
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        // Run until the end of the match (until stop is pressed)
        while (opModeIsActive()) {

            if (gamepad1.a) {
                elevatorCommand = new ElevatorCommand(elevatorSubsystem, ElevatorCommand.ElevatorState.START, telemetry);
                elevatorCommand.execute();
                sleep(500); // Keep running for 500ms (you can adjust this)
            } else if (gamepad1.b) {
                elevatorCommand = new ElevatorCommand(elevatorSubsystem, ElevatorCommand.ElevatorState.STOP, telemetry);
                elevatorCommand.execute();
            } else if (gamepad1.a && gamepad1.b) {
                elevatorCommand = new ElevatorCommand(elevatorSubsystem, ElevatorCommand.ElevatorState.REVERSE, telemetry);
                elevatorCommand.execute();
                sleep(500); // Keep running for 500ms (adjust as needed)
            } else if (gamepad1.x) {
                elevatorCommand = new ElevatorCommand(elevatorSubsystem, ElevatorCommand.ElevatorState.LEFT, telemetry);
                elevatorCommand.execute();
            } else if (gamepad1.y) {
                elevatorCommand = new ElevatorCommand(elevatorSubsystem, ElevatorCommand.ElevatorState.RIGHT, telemetry);
                elevatorCommand.execute();

                // Adding a small delay to prevent overloading the CPU
                sleep(100);
            }
        }
    }
}