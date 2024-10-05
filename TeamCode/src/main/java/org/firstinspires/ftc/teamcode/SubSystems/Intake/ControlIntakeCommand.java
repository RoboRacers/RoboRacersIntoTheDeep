package org.firstinspires.ftc.teamcode.SubSystems.Intake;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ControlIntakeCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final Telemetry telemetry;

    // Enum for intake control state
    public enum IntakeState {
        START, STOP, REVERSE
    }

    private final IntakeState intakeState;

    // Constructor
    public ControlIntakeCommand(IntakeSubsystem intakeSubsystem,
                                IntakeState intakeState, Telemetry telemetry) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeState = intakeState;
        this.telemetry = telemetry;
    }

    // Execute command based on state
    public void execute() {
        switch (intakeState) {
            case START:
                intakeSubsystem.startIntake();
                telemetry.addData("Command", "Intake Started");
                break;

            case STOP:
                intakeSubsystem.stopIntake();
                telemetry.addData("Command", "Intake Stopped");
                break;

            case REVERSE:
                intakeSubsystem.reverseIntake();
                telemetry.addData("Command", "Intake Reversed");
                break;
        }
        telemetry.update();
    }
}