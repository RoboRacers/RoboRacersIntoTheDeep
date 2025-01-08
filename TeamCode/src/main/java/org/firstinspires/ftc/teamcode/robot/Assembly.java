package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.PIDController;

//@TeleOp(name = "LM2 One Driver", group = "Test")
public class Assembly implements Subsystem {

    Servo flipLeft;
    Servo flipRight;

    private Rev2mDistanceSensor distanceSensor;
    public AnalogInput pot;

    Servo claw;
    final double CLAW_OPEN = 0.43;
    final double CLAW_CLOSE = 0.72;

    Servo rotateClaw;

    public double flipPos = 0;

    // Pitch Stuff
    public double pitchTarget = 0;
    public DcMotorImplEx pitchMotor;
    public static double kFPitch = 0.22;
    public static double kPPitch = 0.026;
    public static double kIPitch = 0;
    public static double kDPitch = 0.003;//pitch constant
    public double lastTargetPitch = 0.0;
    public double lastErrorPitch = 0.0;
    public double currentAngle = 0;
    PIDController pitchPID;
    public final double PITCH_LOW_POSITION = 10;
    public final double PITCH_MID_POSITION = 30;
    public final double PITCH_HIGH_POSITION = 110;
    public final double PITCH_POSITION_TOLERANCE = 1;
    public enum PitchPosition {
        DOWN,
        MID,
        HIGH
    }

    // Slides Stuff
    public DcMotorImplEx slidesMotor;
    public double slidesKP = 0.005;
    public double slidesKI = 0; // slides constant
    public double slidesKD = 0.0008;
    public  double offset = 40;
    PIDController slidesControl;
    public  double slidesTarget = 0; //slides
    public  double ticksPerMaxExtend = 1936;
    public  double ticksPerRightAngle = 930;
    final double ticksToInches = (double) 26 /ticksPerMaxExtend;
    final double ticksToDegrees = (double) 90 /ticksPerRightAngle;

    public final int slidesLowPosition = 400;
    public final int slidesMidPosition = 950;
    public final int slidesHighPosition = 1750;
    public final int slidesPositionTolerance = 20;

    public enum SlidesPosition {
        DOWN,
        MID,
        HIGH,
        MANUALUP,
        MANUALDOWN,
        STAY
    }

    public Assembly(HardwareMap hardwareMap) {
        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
        flipLeft = hardwareMap.get(Servo.class, "flipLeft");
        flipRight = hardwareMap.get(Servo.class, "flipRight");
        pitchMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flipLeft = hardwareMap.get(Servo.class, "flipLeft");
        flipRight = hardwareMap.get(Servo.class, "flipRight");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");
        claw = hardwareMap.get(Servo.class, "claw");

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");
        pot = hardwareMap.get(AnalogInput.class,"pot");

        slidesControl = new PIDController(slidesKP, slidesKI, slidesKD);
        pitchPID = new PIDController(kPPitch, kIPitch, kDPitch);
        pitchPID.setOutputLimits(-1, 1);
    }

    /**
     * Conversion for the distance sensor cm to slide encoder position
     * @param pos
     * @return
     */
    private double rawToGoodWithFilterSlides(double pos) {
        double actualPos;
        if(pos>6) {actualPos = 0.0000330052 * (Math.pow(pos, 4)) -0.00356719 * (Math.pow(pos, 3)) + 0.137523 * (Math.pow(pos, 2))  -1.15156*pos + 9.04499;}
        else{actualPos = pos;}
        return Math.round(actualPos);
    }

    /**
     * Potentiaometer to pitch angle
     * @param potentiometerValue
     * @return
     */
    private double mapPotentiometerToAngle(double potentiometerValue) {
        return ((potentiometerValue - 0.47800000000000004)/ (1.1360000000000001-0.47800000000000004)) * (90 - 0) -0;
    }

    public Action flipUp() {
        return telemetryPacket -> {
            flipPos = 0.75;
            flipLeft.setPosition(flipPos);
            flipRight.setPosition(flipPos * 0.94);
            return false;
        };
    }

    public Action flipDown() {
        return telemetryPacket -> {
            flipPos = 0.130;
            flipLeft.setPosition(flipPos);
            flipRight.setPosition(flipPos*0.94);
            return false;
        };
    }

    public Action flipCus(double pos){
        return telemetryPacket -> {
            flipPos = pos;
            flipLeft.setPosition(flipPos);
            flipRight.setPosition(flipPos*0.94);
            return false;
        };
    }

    public Action flipLowMid() {
        return telemetryPacket -> {
            flipPos = 0.330;
            flipLeft.setPosition(flipPos);
            flipRight.setPosition(flipPos*0.94);
            return false;
        };
    }


    public Action flipMid() {
        return telemetryPacket -> {
            flipPos = 0.575;
            flipLeft.setPosition(flipPos);
            flipRight.setPosition(flipPos*0.94);
            return false;
        };
    }

    public Action clawOpen() {
        return telemetryPacket -> {
            claw.setPosition(CLAW_OPEN);
            return false;
        };
    }

    public Action clawClose() {
        return telemetryPacket -> {
            claw.setPosition(CLAW_CLOSE);
            return false;
        };
    }

    public Action rotateClaw(double pos) {
        return telemetryPacket -> {
            rotateClaw.setPosition(pos);
            return false;
        };
    }

//    public Action extendSlide(int position) {
//        return telemetryPacket -> {
//            slidesTarget = position;
//            slidesControl.setSetpoint(slidesTarget);
//            double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
//            double pid2 = slidesControl.calculate(slidesMotor.getCurrentPosition());
//            slidesMotor.setPower(-(pid2 + feedforward));
//
//            double pos = slidesMotor.getCurrentPosition();
//            return pos>(slidesTarget - slidesPositionTolerance) && pos<(slidesTarget + slidesPositionTolerance);
//        };
//    }

//    public Action extendSlide(SlidesPosition position) {
//        return telemetryPacket -> {
//            switch (position) {
//                case DOWN:
//                    slidesTarget = slidesLowPosition;
//                    break;
//                case MID:
//                    slidesTarget = slidesMidPosition;
//                    break;
//                case HIGH:
//                    slidesTarget = slidesHighPosition;
//                    break;
//                case MANUALUP:
//                    slidesTarget+=75;
//                    break;
//                case MANUALDOWN:
//                    slidesTarget-=75;
//                    break;
//                case STAY:
//                    slidesTarget= slidesTarget;
//            }
//            slidesControl.setSetpoint(slidesTarget);
//            double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
//            double pid = slidesControl.calculate(slidesMotor.getCurrentPosition());
//            slidesMotor.setPower(-(pid + feedforward));
//
//            double pos = slidesMotor.getCurrentPosition();
//            return pos>(slidesTarget - slidesPositionTolerance) && pos<(slidesTarget + slidesPositionTolerance);
//        };
//    }

    public void setPitchTarget (double pitchTarget) {
        this.pitchTarget = pitchTarget;
    }

    /**
     *
     * @param targetAngle
     * @return current error
     */
    private void pitchPIDUpdate(double targetAngle) {
        currentAngle = mapPotentiometerToAngle(pot.getVoltage());
        pitchPID.setSetpoint(targetAngle);

        double error = targetAngle - currentAngle;

        if (targetAngle>lastTargetPitch){
            kPPitch = 0.026;
            kDPitch = 0.003;
            kIPitch = 0.000;
            kFPitch = 0.22;
            //
            pitchPID.setCoefficients(kPPitch, kIPitch, kDPitch);
        } else if (targetAngle<lastTargetPitch){
            kPPitch = 0.009;
            kDPitch = 0.0;
            kIPitch = 0.0;
            kFPitch= 0.1;
            //
            pitchPID.setCoefficients(kPPitch, kIPitch, kDPitch);
        }

        if(Math.abs(error) < 10 && Math.abs(error)>1){
            kIPitch = 0.0018;
            pitchPID.setCoefficients(kPPitch, kIPitch, kDPitch);
        }

        double feedforward = kFPitch * Math.cos(Math.toRadians(currentAngle));
        double motorPower = pitchPID.calculate(currentAngle) + feedforward;
        pitchMotor.setPower(-motorPower);

        lastTargetPitch = targetAngle;
    }

    /**
     * Action that runs pitch control
     * @param targetAngle Target
     * @return
     */
    public Action anglePitch(double targetAngle) {
        return (p) -> {
            pitchTarget = targetAngle;
            double currentAngle = mapPotentiometerToAngle(pot.getVoltage());
            pitchPID.setSetpoint(pitchTarget);

            double error = pitchTarget - currentAngle;

            if (targetAngle>lastTargetPitch){
                kPPitch = 0.026;
                kDPitch = 0.003;
                kIPitch = 0.000;
                kFPitch = 0.22;
                //
                pitchPID.setCoefficients(kPPitch, kIPitch, kDPitch);
            } else if (targetAngle<lastTargetPitch){
                kPPitch = 0.009;
                kDPitch = 0.0;
                kIPitch = 0.0;
                kFPitch= 0.1;
                //
                pitchPID.setCoefficients(kPPitch, kIPitch, kDPitch);
            }

            if(Math.abs(error) < 10 && Math.abs(error)>1){
                kIPitch = 0.0018;
                pitchPID.setCoefficients(kPPitch, kIPitch, kDPitch);
            }

            double feedforward = kFPitch * Math.cos(Math.toRadians(currentAngle));
            double motorPower = pitchPID.calculate(currentAngle) + feedforward;
            pitchMotor.setPower(-motorPower);

            lastTargetPitch = targetAngle;

            return Math.abs(error) < PITCH_POSITION_TOLERANCE;
        };
    }

    @Override
    public void update() {
        // Pitch PID Update
        pitchPIDUpdate(pitchTarget);
        currentAngle = mapPotentiometerToAngle(pot.getVoltage());
        // Slides Code

    }

    public void slidesManual(double value) {
            slidesMotor.setPower(value);
    }

}