package org.firstinspires.ftc.teamcode.SubSystems.Elevator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.ControlIntakeCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;

public class ElevatorCommand {


    private final ElevatorSubsystem elevatorSubsystem;
    private final Telemetry telemetry;

    // Enum for intake control state
    public enum ElevatorState {
        START, STOP, REVERSE, LEFT, RIGHT
    }

    private final ElevatorState elevatorState;

    // Constructor
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem,
                             ElevatorCommand.ElevatorState elevatorState, Telemetry telemetry) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorState = elevatorState;
        this.telemetry = telemetry;
    }

    // Execute command based on state
    public void execute() {
        switch (elevatorState) {
            case START:
                elevatorSubsystem.moveUp();
                telemetry.addData("Command", "Intake Started");
                break;

            case STOP:
                elevatorSubsystem.stopAction();
                telemetry.addData("Command", "Intake Stopped");
                break;

            case REVERSE:
                elevatorSubsystem.moveDown();
                telemetry.addData("Command", "Intake Reversed");
                break;
        }
        telemetry.update();
    }
}

