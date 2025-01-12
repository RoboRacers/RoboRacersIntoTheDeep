package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.PIDController;

@Config
public class Assembly implements Subsystem {

    /*
     * Servos
     */
    public Servo flipLeft;
    public Servo flipRight;
    public double flipPos = 0;
    public Servo rotateClaw;
    public Servo claw;
    private boolean CLAW_STATE;
    final double CLAW_OPEN = 0.01;
    final double CLAW_CLOSE = 0.2;
    /*
     * Sensors
     */
    private Rev2mDistanceSensor distanceSensor;
    public AnalogInput pot;
    /*
     * Pitch variables
     */
    public DcMotorImplEx pitchMotor;
    // PID Constants
    PIDController pitchPID;
    /* Old Constants
     *  kp = 0.026
     *  ki = 0
     *  kd = 0.003
     *  kf (not used) = 0.22
     */
    public static double pitchKp = 0.002;
    public static double pitchKi = 0;
    public static double pitchKd = 0.000001;//pitch constant
    public static double pitchKf = 0;
    public static double pitchTarget = 0;
    private double pitchAngle = 0;
    public double lastPitchTarget = 0.0;
    public double lastPitchError = 0.0;
    public boolean pitchPIDEnabled = true;
    public double getPitchAngle() {return pitchAngle;}
    // Preset positions
    public static final double PITCH_LOW_POSITION = -200;
    public static final double PITCH_MID_POSITION = -600;
    public static final double PITCH_AUTO_POSITION = -400;
    public static final double PITCH_HIGH_POSITION = -2200;
    public static final double PITCH_POSITION_TOLERANCE = 1;

    public enum PitchPosition {
        LOW(10),
        MID(30),
        HIGH(110);

        public final double position;

        PitchPosition(double position) {
            this.position = position;
        }
    }
    /*
     * Slides variables
     */
    public DcMotorImplEx slidesMotor;
    // PID Constants
    PIDController slidesPID;
    public static double slidesKp = 0.005;
    public static double slidesKi = 0; // slides constant
    public static double slidesKd = 0.0008;
//    public double slidesKP = 0.042;
//public double slidesKP = 0.047;
//public double slidesKP = 0.052;
    public double slidesKP = 0.072;
    public double slidesKI = 0.001; // slides constant
    public double slidesKD = 0.002;
    public double slidesKF = 0.39;
    public static double offset = 40;
    public int slideTarget = 0;
    public static double slidePosition = 0;
    // Tick tuning values
    public double ticksPerMaxExtend = 1936;
    public double ticksPerRightAngle = 930;
    final double ticksToInches = (double) 26 /ticksPerMaxExtend;
    final double ticksToDegrees = (double) 90 /ticksPerRightAngle;
    // Preset positions
    public static final int SLIDES_LOW_POSITION = 10;
    public static final int SLIDES_MID_POSITION = 25;
    public static final int SLIDES_AUTO_POSITION = 40;
    public static final int SLIDES_HIGH_POSITION = 70;
    public static final int SLIDES_POSITION_TOLERANCE = 20;
    public enum SlidesPosition {
        LOW,
        MID,
        HIGH
    }

    public Assembly(HardwareMap hardwareMap) {
        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");

        pitchMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flipLeft = hardwareMap.get(Servo.class, "flipLeft");
        flipRight = hardwareMap.get(Servo.class, "flipRight");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_CLOSE);

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");
        pot = hardwareMap.get(AnalogInput.class,"pot");

        slidesPID = new PIDController(slidesKP, slidesKI, slidesKD);
        slidesPID.setOutputLimits(-1,1);
        pitchPID = new PIDController(pitchKp, pitchKi, pitchKd);
        pitchPID.setOutputLimits(-1, 1);
    }

    /**
     * Conversion for the distance sensor cm to slide encoder position
     * @param pos
     * @return
     */
    private double distanceSensorToTicks(double pos) {
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
            CLAW_STATE = true;
            return false;
        };
    }

    public Action clawClose() {
        return telemetryPacket -> {
            claw.setPosition(CLAW_CLOSE);
            CLAW_STATE = false;
            return false;
        };
    }

    public void toggleClaw() {
        if (CLAW_STATE) {
            claw.setPosition(CLAW_CLOSE);
        } else {
            claw.setPosition(CLAW_OPEN);
        }
        CLAW_STATE = !CLAW_STATE;
    }

    public Action rotateClaw(double pos) {
        return telemetryPacket -> {
            rotateClaw.setPosition(pos);
            return false;
        };
    }

    public void setPitchTarget (double pitchTarget) {
        Assembly.pitchTarget = pitchTarget;
    }

    public void setSlideTarget (int slideTarget) {
        slideTarget = slideTarget;
    }

    /**
     *
     * @param targetAngle
     * @return current error
     */
    private void pitchPIDUpdate(double targetAngle) {
        pitchAngle = mapPotentiometerToAngle(pot.getVoltage());
        pitchPID.setSetpoint(targetAngle);

        pitchPID.setCoefficients(pitchKp, pitchKi, pitchKd);

        double feedforward = pitchKf * Math.cos(Math.toRadians(pitchAngle));
        double motorPower = pitchPID.calculate(pitchAngle) + feedforward;
        pitchMotor.setPower(-motorPower);

        lastPitchTarget = targetAngle;
    }

    /**
     *
     * @param targetTicks
     * @return current error
     */
    private void pitchPIDUpdate(int targetTicks) {
        int currentPitchTicks = pitchMotor.getCurrentPosition();
        pitchPID.setSetpoint(targetTicks);

        pitchPID.setCoefficients(pitchKp, pitchKi, pitchKd);

//        double feedforward = pitchKf * Math.cos(Math.toRadians(pitchAngle));
        double motorPower = pitchPID.calculate(currentPitchTicks);
        pitchMotor.setPower(motorPower);

    }

    /**
     * Action that runs pitch control
     * @param targetAngle Target
     * @return
     */
    public Action anglePitch(double targetAngle) {
        return (p) -> {
            pitchPIDUpdate(targetAngle);
            return Math.abs(pitchTarget - pitchAngle) > PITCH_POSITION_TOLERANCE;
        };
    }

    public void slidesPIDUpdate(int slideTarget) {
        slidePosition = slidesMotor.getCurrentPosition();
        slidesPID.setSetpoint(slideTarget);
        double motorPower = slidesPID.calculate(distanceSensor.getDistance(DistanceUnit.CM));
        double feedForward = slidesKF * Math.sin(Math.toRadians(mapPotentiometerToAngle(pot.getVoltage())));
        motorPower = motorPower + feedForward;
        slidesMotor.setPower(-motorPower);
    }

    public Action extendSlide(int slideTarget) {
        return telemetryPacket -> {
            //slideTarget = slideTarget;
            slidePosition = slidesMotor.getCurrentPosition();
            slidesPIDUpdate(slideTarget);
            return Math.abs(slideTarget - slidePosition) > SLIDES_POSITION_TOLERANCE;
        };
    }

    public void setPIDEnable(boolean enable) {
        this.pitchPIDEnabled = enable;
    }

    @Override
    public void update() {
        // Pitch PID Update
        if (pitchPIDEnabled) {
            pitchPIDUpdate((int) pitchTarget);
        }
        pitchAngle = mapPotentiometerToAngle(pot.getVoltage());

        // Slides Code
        slidesPIDUpdate(slideTarget);

    }

    public void slidesManual(double value) {
            slidesMotor.setPower(value);
    }

}