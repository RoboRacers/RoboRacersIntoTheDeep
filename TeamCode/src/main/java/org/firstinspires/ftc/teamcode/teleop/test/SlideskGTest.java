package org.firstinspires.ftc.teamcode.teleop.test;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.PIDController;

@Config
@TeleOp(name = "Slides kG Test", group = "Test")
public class SlideskGTest extends LinearOpMode {
    public DcMotorImplEx slidesMotor;
    public DcMotorImplEx pitchMotor;

    public static double kG = 0.35;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    PIDController slidesControl;

    @Override
    public void runOpMode() throws InterruptedException {

        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");

        double target = 0;

        slidesControl = new PIDController(kP, kI, kD);

        final double ticksToDegrees = (double) 90 /334;

        while (opModeInInit()) {
        }

        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStopRequested()) {

            target = slidesMotor.getCurrentPosition();

            slidesControl.setSetpoint(target);

            double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - 45) * ticksToDegrees)) + 0;

            telemetry.addData("Feedforward", feedforward);

            slidesMotor.setPower(gamepad1.right_stick_x*feedforward);

            telemetry.addData("Pitch Motor Position", slidesMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", slidesMotor.getPower());
            telemetry.addData("Pitch Current", slidesMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Pitch Motor Angle", (slidesMotor.getCurrentPosition() - 45 ) * ticksToDegrees);
            telemetry.update();

        }
    }
}
