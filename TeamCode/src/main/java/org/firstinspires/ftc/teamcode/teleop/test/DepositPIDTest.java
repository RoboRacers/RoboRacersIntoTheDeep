package org.firstinspires.ftc.teamcode.teleop.test;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.PIDController;

@Config
@TeleOp(name = "Pitch PID Test", group = "Test")
public class DepositPIDTest extends LinearOpMode {
    public DcMotorImplEx pitchMotor;

    public static double kG = 0.27;
    public static double kP = 0.005;
    public static  double kI = 0;
    public static  double kD = 0.00045;
    public static double target = 100;

    public static double offset = 40;
    public static double error = 0;

    public static double ticksPerRightAngle = 930;

    PIDController pitchControl;

    @Override
    public void runOpMode() throws InterruptedException {

        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        pitchControl = new PIDController(kP, kI, kD);

        final double ticksToDegrees = (double) 90 /ticksPerRightAngle;

        while (opModeInInit()) {

        }

        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStopRequested()) {

            pitchControl.setCoefficients(kP, kI, kD);

            if (gamepad1.triangle) {
                target = 300;
            } else if (gamepad1.cross) {
                target = 200;
            } else if (gamepad1.circle) {
                target = 150;
            }

            pitchControl.setSetpoint(target);

            double feedforward = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;

            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());

            pitchMotor.setPower(feedforward + pid);

            // Compute error as the difference between current position and target
            double currentPosition = pitchMotor.getCurrentPosition();
            error = target - currentPosition;
            
            telemetry.addData("Feedforward", feedforward);
            telemetry.addData("Target", target);
            telemetry.addData("Error", error);
            telemetry.addData("Pitch Motor Position", pitchMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", pitchMotor.getPower());
            telemetry.addData("Pitch Current", pitchMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Pitch Motor Angle", (pitchMotor.getCurrentPosition() - 45 ) * ticksToDegrees);
            telemetry.update();

        }
    }
}
