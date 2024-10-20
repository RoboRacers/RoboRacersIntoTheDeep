package org.firstinspires.ftc.teamcode.SubSystems.Elevator;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElevatorSubsystem {

    private final Servo elevatorMotor;
    private final Telemetry telemetry;
    public ElevatorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        elevatorMotor = hardwareMap.get(Servo.class, "left_hand");
    }

    public void moveUp() {
        elevatorMotor.setPosition(1.0); // Set the servo to 270 degrees
        telemetry.addData("Elevator", "Running");
        telemetry.update();
    }

    public void stopAction() {
        elevatorMotor.setPosition(0.0); // Stop the motor
        telemetry.addData("Elevator", "Stopped");
        telemetry.update();
    }
    public void moveDown() {
        elevatorMotor.setPosition(0.5); // Set motor to reverse
        telemetry.addData("Elevator", "Reversing");
        telemetry.update();
    }
}