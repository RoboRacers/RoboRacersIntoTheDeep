package org.firstinspires.ftc.teamcode.teleop;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.PIDController;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

@Config
//@TeleOp(name = "Slides PID Test", group = "Test")
public class DepositSlidesSubsystem implements Subsystem {
    public DcMotorImplEx slidesMotor;
    public DcMotorImplEx pitchMotor;

    public static double kG = 0.027;
    public static double kG2 = 0.1;
    public static double kP = 0.005;
    public static  double kI = 0;
    public static  double kD = 0.00045;
    public static double kP2 = 0.005;
    public static double kI2 = 0;
    public static double kD2 = 0.0008;
    public static double ticksPerRightAngle = 930;
    public static double ticksPerMaxExtend = 1936;
    public static double target = 100;
    public static double target2 = 300;


    PIDController slidesControl;
    PIDController pitchControl;
    public static double offset = 40;

    @Override
    public void update() {

//        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
//        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
//        slidesMotor.setDirection(DcMotorImplEx.Direction.REVERSE);




        slidesControl = new PIDController(kP2, kI2, kD2);
        pitchControl = new PIDController(kP, kI, kD);

        final double ticksToDegrees = (double) 90 /ticksPerRightAngle;
        final double ticksToInches = (double) 26 /ticksPerMaxExtend;



        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            pitchControl.setCoefficients(kP, kI, kD);
            slidesControl.setCoefficients(kP2, kI2, kD2);


            pitchControl.setSetpoint(target2);

            slidesControl.setSetpoint(target);

            double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double feedforward2 = kG2 * Math.sin(Math.toRadians((slidesMotor.getCurrentPosition()) * ticksToInches)) + 0;
            double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
            double pid2 = slidesControl.calculate(slidesMotor.getCurrentPosition());

//            telemetry.addData("Feedforward", feedforward);

            pitchMotor.setPower(feedforward3 + pid + feedforward2);
//            slidesMotor.setPower(-(pid2 + feedforward +(gamepad1.left_stick_x *0.5)));
//            telemetry.addData("slide power", slidesMotor.getPower());
//            telemetry.addData("Pitch Motor Position", slidesMotor.getCurrentPosition());
//            telemetry.addData("Pitch Motor Power", slidesMotor.getPower());
//            telemetry.addData("Pitch Current", slidesMotor.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("Pitch Motor Angle", (slidesMotor.getCurrentPosition() - 45 ) * ticksToDegrees);
//            telemetry.update();


    }
}
