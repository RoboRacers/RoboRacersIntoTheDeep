package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@Config
@TeleOp(name = "Slides Custom PID Control", group = "Examples")
public class slidesPID extends LinearOpMode {

    // PID coefficients (modifiable from FTC Dashboard)
    public static double Kp = 0.002;
    public static double Ki = 0.0001;
    public static double Kd = 0.01;

    // Target position in encoder ticks, also adjustable via Dashboard
    public static int TARGET_POSITION = 1000;

    // Variables to track PID calculations
    private double integralSum = 0;
    private double lastError = 0;

    @Override
    public void runOpMode() {
        // Initialize the motor
        DcMotorImplEx slideMotor = hardwareMap.get(DcMotorImplEx.class, "Slides_Left"); //Slides_Left
        //slideMotor = hardwareMap.get(DcMotor.class, "Horizontal_Slides");


        // Reset and set the motor mode
//        slideMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);

        // Initialize FTC Dashboard for telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();

        while (opModeInInit()) {
            //slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        

        waitForStart();


        while (opModeIsActive()) {

            // Calculate the error
            double currentPosition = slideMotor.getCurrentPosition();
            double error = TARGET_POSITION - slideMotor.getCurrentPosition();

            // Calculate the integral and derivative terms
            integralSum += error;
            double derivative = error - lastError;

            // Calculate the control signal (motor power)
            double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            // Apply power to the motor
            slideMotor.setPower(power);

            // Update the last error
            lastError = error;

            // Send telemetry to FTC Dashboard
            telemetry.addData("Current Position", currentPosition);

            telemetry.addData("Real Position", slideMotor.getCurrentPosition());

            telemetry.addData("Real Power Position", slideMotor.getPower());
            telemetry.addData("Target Position", TARGET_POSITION);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.update();
        }
    }
}
