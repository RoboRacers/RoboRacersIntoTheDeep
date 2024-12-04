package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.PIDController;

public class Pitch implements Subsystem {
    public static double kG = 0.027;
    public static double kG2 = 0.001;
    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0.00045;//pitch constant
    public static double ticksPerRightAngle = 930;
    public static double ticksPerMaxExtend = 1936;
    public static double target2 = 100; // flip
    public static double offset = 40;
    public static double flipPos = 0;
    Servo uno;
    Servo dos;


    public DcMotorImplEx pitchMotor;
    PIDController pitchControl;
    public DcMotorImplEx slidesMotor;

    public FlipHighBasket(HardwareMap hardwareMap){

        final double ticksToDegrees = (double) 90 /ticksPerRightAngle;
        final double ticksToInches = (double) 26 /ticksPerMaxExtend;

        flipPos = 0.78;
        pitchControl.setCoefficients(kP, kI, kD);
        uno.setPosition(flipPos);
        dos.setPosition(flipPos * 0.94);
        target2 = 250;


        pitchControl.setSetpoint(target2);


        double feedforward2 = kG2 * Math.sin(Math.toRadians((slidesMotor.getCurrentPosition()) * ticksToInches)) + 0;
        double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
        double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
        pitchMotor.setPower(feedforward3 + pid + feedforward2);

        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
        pitchControl = new PIDController(kP, kI, kD);

        pitchMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);

        pitchMotor.setPower(feedforward3 + pid + feedforward2);

        target2 = 1010; //90deg + little more
        flipPos = 0.525;

        pitchControl.setSetpoint(target2);

        return();

    }


    @Override
    public void update() {

    }
}
