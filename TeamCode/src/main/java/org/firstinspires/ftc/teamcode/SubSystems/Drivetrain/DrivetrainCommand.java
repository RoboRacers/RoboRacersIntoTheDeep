package org.firstinspires.ftc.teamcode.SubSystems.Drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.ControlIntakeCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;

public class DrivetrainCommand {


    private final DrivetrainSubsystem drivetrainSubsystem;
    private final Telemetry telemetry;

    // Enum for intake control state
    public enum DrivetrainState {
        START, STOP, REVERSE, LEFT, RIGHT
    }

    private final DrivetrainState drivetrainState;

    // Constructor
    public DrivetrainCommand(DrivetrainSubsystem drivetrainSubsystem,
                             DrivetrainCommand.DrivetrainState drivetrainState, Telemetry telemetry) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.drivetrainState = drivetrainState;
        this.telemetry = telemetry;
    }

    // Execute command based on state
    public void execute() {
        switch (drivetrainState) {
            case START:
                drivetrainSubsystem.startMoving();
                telemetry.addData("Command", "Intake Started");
                break;

            case STOP:
                drivetrainSubsystem.stopMoving();
                telemetry.addData("Command", "Intake Stopped");
                break;

            case REVERSE:
                drivetrainSubsystem.moveBackwards();
                telemetry.addData("Command", "Intake Reversed");
                break;


        }
        telemetry.update();
    }
}
