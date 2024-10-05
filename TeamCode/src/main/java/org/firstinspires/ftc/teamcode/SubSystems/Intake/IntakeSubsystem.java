package org.firstinspires.ftc.teamcode.SubSystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {

    private final Servo intakeMotor; // Motor controlling the intake
    private final Telemetry telemetry; // For debug purposes

    // Constructor
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(Servo.class, "left_hand");
    }

    // Start the intake (forwards)
    public void startIntake() {
        intakeMotor.setPosition(1.0); // Set the servo to 270 degrees
        telemetry.addData("Intake", "Running");
        telemetry.update();
    }

    // Stop the intake
    public void stopIntake() {
        intakeMotor.setPosition(0.0); // Stop the motor
        telemetry.addData("Intake", "Stopped");
        telemetry.update();
    }

    // Reverse the intake (for ejecting)
    public void reverseIntake() {
        intakeMotor.setPosition(0.5); // Set motor to reverse
        telemetry.addData("Intake", "Reversing");
        telemetry.update();
    }
}