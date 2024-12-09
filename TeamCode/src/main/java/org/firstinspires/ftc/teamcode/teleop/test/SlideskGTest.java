package org.firstinspires.ftc.teamcode.teleop.test;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.PIDController;

@Config
@Disabled
@TeleOp(name = "Slides kG Test", group = "Test")
public class SlideskGTest extends LinearOpMode {
    public DcMotorImplEx slidesMotor;
    public DcMotorImplEx pitchMotor;

    public static double kG = 0.10;
    public static double kG2 = 0.27;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double ticksPerRightAngle = 930;


    PIDController slidesControl;
    PIDController pitchControl;
    public static double offset = 40;

    @Override
    public void runOpMode() throws InterruptedException {

        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
slidesMotor.setDirection(DcMotorImplEx.Direction.REVERSE);
        double target = 0;

        double target2 = 0;

        slidesControl = new PIDController(kP, kI, kD);
        pitchControl = new PIDController(kP, kI, kD);

        final double ticksToDegrees = (double) 90 /ticksPerRightAngle;

        while (opModeInInit()) {
        }

        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStopRequested()) {

            target = slidesMotor.getCurrentPosition();

            target2 = pitchMotor.getCurrentPosition();

            pitchControl.setSetpoint(target2);

            slidesControl.setSetpoint(target);

            double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;

            double feedforward2 = kG2 * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;


            telemetry.addData("Feedforward", feedforward);

            slidesMotor.setPower(gamepad1.right_stick_x+feedforward);
            pitchMotor.setPower(feedforward2);

            telemetry.addData("Pitch Motor Position", slidesMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", slidesMotor.getPower());
            telemetry.addData("Pitch Current", slidesMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Pitch Motor Angle", (slidesMotor.getCurrentPosition() - 45 ) * ticksToDegrees);
            telemetry.update();

        }
    }
}
