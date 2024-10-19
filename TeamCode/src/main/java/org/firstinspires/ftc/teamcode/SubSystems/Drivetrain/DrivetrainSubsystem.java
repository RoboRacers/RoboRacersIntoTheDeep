package org.firstinspires.ftc.teamcode.SubSystems.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DrivetrainSubsystem {

    private final DcMotor bottomLeftMotor; // Motor controlling the intake
    private final DcMotor bottomRightMotor;
    private final DcMotor topLeftMotor;
    private final DcMotor topRightMotor;
    private final Telemetry telemetry; // For debug purposes

    // Constructor
    public DrivetrainSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        bottomLeftMotor = hardwareMap.get(DcMotor.class,"Drivetrain");
        bottomRightMotor= hardwareMap.get(DcMotor.class,"Drivetrain");
        topRightMotor= hardwareMap.get(DcMotor.class,"Drivetrain");
        topLeftMotor= hardwareMap.get(DcMotor.class,"Drivetrain");
    }

    // Start the intake (forwards)
    public void startMoving() {
        bottomLeftMotor.setPower(1.0);
        bottomRightMotor.setPower(1.0);
        topRightMotor.setPower(1.0);
        topLeftMotor.setPower(1.0);

        telemetry.addData("Intake", "Running");
        telemetry.update();
    }

    // Stop the intake
    public void stopMoving() {
        bottomLeftMotor.setPower(0.0);
        bottomRightMotor.setPower(0.0);
        topRightMotor.setPower(0.0);
        topLeftMotor.setPower(0.0);

        telemetry.addData("Intake", "Running");
        telemetry.update();
    }

    // Reverse the intake (for ejecting)
    public void moveBackwards() {
        bottomLeftMotor.setPower(-1.0);
        bottomRightMotor.setPower(-1.0);
        topRightMotor.setPower(-1.0);
        topLeftMotor.setPower(-1.0);

        telemetry.addData("Intake", "Running");
        telemetry.update();
    }
    public void turnLeft(){
        bottomLeftMotor.setPower(0.5);
        bottomRightMotor.setPower(1.0);
        topRightMotor.setPower(0.5);
        topLeftMotor.setPower(1.0);

        telemetry.addData("Intake", "Running");
        telemetry.update();

    }
    public void turnRight() {
        bottomLeftMotor.setPower(1.0);
        bottomRightMotor.setPower(0.5);
        topRightMotor.setPower(1.0);
        topLeftMotor.setPower(0.5);

        telemetry.addData("Intake", "Running");
        telemetry.update();

    }
}
