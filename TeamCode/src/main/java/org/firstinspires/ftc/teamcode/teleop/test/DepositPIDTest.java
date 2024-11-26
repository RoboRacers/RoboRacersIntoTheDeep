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

    public static double kG = 0.35;
    public static double kP = 0.5;
    public static  double kI = 0;
    public static  double kD = 0;

    PIDController pitchControl;

    @Override
    public void runOpMode() throws InterruptedException {

        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");

        double target = 0;

        pitchControl = new PIDController(kP, kI, kD);

        final double ticksToDegrees = (double) 90 /260;

        while (opModeInInit()) {

        }

        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStopRequested()) {

            pitchControl.setCoeffiecents(kP, kI, kD);

            if (gamepad1.triangle) {
                target = 300;
            } else if (gamepad1.cross) {
                target = 200;
            } else if (gamepad1.circle) {
                target = 100;
            }

            pitchControl.setSetpoint(target);

            double feedforward = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - 20) * ticksToDegrees)) + 0;

            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());

            pitchMotor.setPower(feedforward + pid);
            
            telemetry.addData("Feedforward", feedforward);

            telemetry.addData("Pitch Motor Position", pitchMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", pitchMotor.getPower());
            telemetry.addData("Pitch Current", pitchMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Pitch Motor Angle", (pitchMotor.getCurrentPosition() - 20 ) * ticksToDegrees);
            telemetry.update();

        }
    }
}
