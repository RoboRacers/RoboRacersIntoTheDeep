package org.firstinspires.ftc.teamcode.teleop.test;



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

    double kG = 0.2444;
    public static double kP = 0.1;
    public static  double kI = 0;
    public static  double kD = 0.1;

    PIDController pitchControl;

    @Override
    public void runOpMode() throws InterruptedException {

        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");

        double target = 0;

        pitchControl = new PIDController(kP, kI, kD);

        final double ticksToDegrees = (double) 90 /371.0;

        while (opModeInInit()) {
            // 175
            // 420
        }

        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStopRequested()) {

            pitchControl.setCoeffiecents(kP, kI, kD);




            if (gamepad1.triangle) {
                target = 420;
            } else if (gamepad1.cross) {
                target = 175;
            } else if (gamepad1.circle) {
                target = 30;
            }

            pitchControl.setSetpoint(target);

            double feedforward = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() + 53) * ticksToDegrees)) + 0.1;
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());

            pitchMotor.setPower(feedforward + pid);

            telemetry.addData("Pitch Motor Position", pitchMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", pitchMotor.getPower());
            telemetry.addData("Pitch Current", pitchMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();

        }
    }
}
