package SubSystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {

    private final DcMotor intakeMotor; // Motor controlling the intake
    private final Telemetry telemetry; // For debug purposes

    // Constructor
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Optionally configure the motor (e.g., direction, mode)
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Start the intake (forwards)
    public void startIntake() {
        intakeMotor.setPower(1.0); // Set the motor to full power
        telemetry.addData("Intake", "Running");
        telemetry.update();
    }

    // Stop the intake
    public void stopIntake() {
        intakeMotor.setPower(0.0); // Stop the motor
        telemetry.addData("Intake", "Stopped");
        telemetry.update();
    }

    // Reverse the intake (for ejecting)
    public void reverseIntake() {
        intakeMotor.setPower(-1.0); // Set motor to reverse
        telemetry.addData("Intake", "Reversing");
        telemetry.update();
    }
}