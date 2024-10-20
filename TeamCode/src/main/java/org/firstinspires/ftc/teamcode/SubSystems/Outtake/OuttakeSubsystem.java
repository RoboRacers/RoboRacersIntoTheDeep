package org.firstinspires.ftc.teamcode.SubSystems.Outtake;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeSubsystem {

    private final Servo outtakeServo; // Servo controlling the outtake
    private final Servo outtakeServo2;
    private final Telemetry telemetry; // For debug purposes

    // Constructor
    public OuttakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        outtakeServo = hardwareMap.get(Servo.class, "left_hand");
        outtakeServo2 = hardwareMap.get(Servo.class, "right_hand"); // Changed to right hand for clarity
    }

    // Start the outtake (ejecting forwards)
    public void startOuttake() {
        outtakeServo.setPosition(1.0); // Set the servo to eject
        outtakeServo2.setPosition(1.0);
        telemetry.addData("Outtake", "Running");
        telemetry.update();
    }

    // Stop the outtake
    public void stopOuttake() {
        outtakeServo.setPosition(0.0); // Stop the outtake
        outtakeServo2.setPosition(0.0);
        telemetry.addData("Outtake", "Stopped");
        telemetry.update();
    }

    // Reverse the outtake (for resetting or pulling in)
    public void reverseOuttake() {
        outtakeServo.setPosition(0.5); // Set servo to reverse
        outtakeServo2.setPosition(0.5);
        telemetry.addData("Outtake", "Reversing");
        telemetry.update();
    }
}
