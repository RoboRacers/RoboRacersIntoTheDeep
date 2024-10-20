package org.firstinspires.ftc.teamcode.SubSystems.Outtake;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeCommand {

    private final OuttakeSubsystem outtakeSubsystem;
    private final Telemetry telemetry;

    // Enum for outtake control state
    public enum OuttakeState {
        START, STOP, REVERSE
    }

    private final OuttakeState outtakeState;

    // Constructor
    public OuttakeCommand(OuttakeSubsystem outtakeSubsystem,
                                 OuttakeState outtakeState, Telemetry telemetry) {
        this.outtakeSubsystem = outtakeSubsystem;
        this.outtakeState = outtakeState;
        this.telemetry = telemetry;
    }

    // Execute command based on state
    public void execute() {
        switch (outtakeState) {
            case START:
                outtakeSubsystem.startOuttake();
                telemetry.addData("Command", "Outtake Started");
                break;

            case STOP:
                outtakeSubsystem.stopOuttake();
                telemetry.addData("Command", "Outtake Stopped");
                break;

            case REVERSE:
                outtakeSubsystem.reverseOuttake();
                telemetry.addData("Command", "Outtake Reversed");
                break;
        }
        telemetry.update();
    }
}
