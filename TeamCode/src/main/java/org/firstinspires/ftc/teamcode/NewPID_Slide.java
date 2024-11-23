package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="PID Motor Control", group="Linear Opmode")
public class NewPID_Slide extends LinearOpMode {

    private DcMotor motor;
    private DcMotor motorDOS;
    private static final double KP = 0.02; // Proportional constant
    private static final double KI = 0.001; // Integral constant
    private static final double KD = 0.01; // Derivative constant
    private static final double MAX_POWER = 1.0; // Maximum motor power
    private static final double TOLERANCE = 5; // Tolerance for position error

    private int targetPosition = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "Slides_Left");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDOS = hardwareMap.get(DcMotor.class, "Slides_Right");
        motorDOS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDOS.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            // Set your target position, e.g., from gamepad input or pre-set value
            targetPosition = 1000; // Example target

            double power = pidControl(targetPosition);
            motor.setPower(power);
            motorDOS.setPower(power);

            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current Position 1", motor.getCurrentPosition());
            telemetry.addData("Current Position 2", motorDOS.getCurrentPosition());
            ;
            telemetry.addData("Current Power 1", motor.getPower());
            telemetry.addData("Current Power 2", motorDOS.getPower());
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }

    private double pidControl(int target) {
        int currentPos = (motor.getCurrentPosition()+motorDOS.getCurrentPosition())/2;
        double error = target - currentPos;

        // Proportional term
        double proportional = KP * error;

        // Integral term
        integralSum += error * timer.seconds();
        double integral = KI * integralSum;

        // Derivative term
        double derivative = KD * (error - lastError) / timer.seconds();
        lastError = error;

        // Reset timer for accurate delta time calculation
        timer.reset();

        // Calculate final power
        double power = proportional + integral + derivative;

        // Limit the power to avoid exceeding motor capabilities
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        // If error is within tolerance, stop integrating
        if (Math.abs(error) < TOLERANCE) {
            integralSum = 0;
            power = 0;
        }

        return power;
    }
}
