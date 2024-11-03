package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    private DcMotorImplEx slideMotorLeft;
    private DcMotorImplEx slideMotorRight;


    @Override
    public void runOpMode() {
        // Initialize the motor
        slideMotorLeft = hardwareMap.get(DcMotorImplEx.class, "Slides_Left"); //Slides_Left
        slideMotorRight = hardwareMap.get(DcMotorImplEx.class, "Slides_Right"); //Slides_Left
        slideMotorRight.setDirection(DcMotorImplEx.Direction.REVERSE);
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
            double currentPosition = (slideMotorLeft.getCurrentPosition()+slideMotorRight.getCurrentPosition())/2.0;
            telemetry.addData("Left Position", slideMotorLeft.getCurrentPosition());
            telemetry.addData("Right Position", slideMotorRight.getCurrentPosition());
            telemetry.addData("Code Position", currentPosition);

            double error = TARGET_POSITION - currentPosition;
            telemetry.addData("Error", error);

            // Calculate the integral and derivative terms
            integralSum += error;
            telemetry.addData("Integral Sum", integralSum);
            double derivative = error - lastError;
            telemetry.addData("derivative", derivative);

            // Calculate the control signal (motor power)
            double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            telemetry.addData("power", power);

            // Apply power to the motor
            slideMotorLeft.setPower(power);
            slideMotorRight.setPower(power);

            // Update the last error
            lastError = error;

            // Send telemetry to FTC Dashboard
            telemetry.addData("Last error", lastError);

            telemetry.addData("Right Power", slideMotorRight.getPower());

            telemetry.addData("LeftPower", slideMotorLeft.getPower());
            telemetry.addData("Target Position", TARGET_POSITION);


            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.update();
        }
    }
}
