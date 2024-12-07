package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.PIDController;

//@TeleOp(name = "LM2 One Driver", group = "Test")
public class Slides implements Subsystem {

    Servo flipLeft;
    Servo flipRight;

    Servo claw;
    final double clawOpen = 0.175;
    final double clawClose = 0.450;

    Servo rotateClaw;

    public double flipPos = 0;

    // Pitch Stuff
    public int pitchTarget = 0;
    public DcMotorImplEx pitchMotor;
    public  double kG = 0.027;
    public static double kG2 = 0.001;
    public static double kP = 0.005;
    public static  double kI = 0;
    public static  double kD = 0.00045;//pitch constant
    PIDController pitchControl;

    public final int pitchLowPosition = 300;
    public final int pitchMidPosition = 600;
    public final int pitchHighPosition = 1000;
    public final int pitchPositionTolerance = 20;

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
    public final int slidesMidPosition = 800;
    public final int slidesHighPosition = 1650;
    public final int slidesPositionTolerance = 20;

    public enum SlidesPosition {
        DOWN,
        MID,
        HIGH
    }

    public Slides(HardwareMap hardwareMap) {
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

        slidesControl = new PIDController(slidesKP, slidesKI, slidesKD);
        pitchControl = new PIDController(kP, kI, kD);

    }


    public Action flipUp() {
        return telemetryPacket -> {
            flipPos = 0.8;
            flipLeft.setPosition(flipPos);
            flipRight.setPosition(flipPos * 0.94);
            return false;
        };
    }

    public Action flipDown() {
        return telemetryPacket -> {
            flipPos = 0.25;
            flipLeft.setPosition(flipPos);
            flipRight.setPosition(flipPos*0.94);
            return false;
        };
    }

    public Action clawOpen() {
        return telemetryPacket -> {
            claw.setPosition(clawOpen);
            return false;
        };
    }

    public Action clawClose() {
        return telemetryPacket -> {
            claw.setPosition(clawClose);
            return false;
        };
    }

    public Action rotateClaw(double pos) {
        return telemetryPacket -> {
            rotateClaw.setPosition(pos);
            return false;
        };
    }

    public Action extendSlide(int position) {
        return telemetryPacket -> {
            slidesTarget = position;
            slidesControl.setSetpoint(slidesTarget);
            double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double pid2 = slidesControl.calculate(slidesMotor.getCurrentPosition());
            slidesMotor.setPower(-(pid2 + feedforward));

            double pos = slidesMotor.getCurrentPosition();
            return pos>(slidesTarget - slidesPositionTolerance) && pos<(slidesTarget + slidesPositionTolerance);
        };
    }

    public Action extendSlide(SlidesPosition position) {
        return telemetryPacket -> {
            switch (position) {
                case DOWN:
                    slidesTarget = slidesLowPosition;
                    break;
                case MID:
                    slidesTarget = slidesMidPosition;
                    break;
                case HIGH:
                    slidesTarget = slidesHighPosition;
                    break;
            }
            slidesControl.setSetpoint(slidesTarget);
            double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double pid = slidesControl.calculate(slidesMotor.getCurrentPosition());
            slidesMotor.setPower(-(pid + feedforward));

            double pos = slidesMotor.getCurrentPosition();
            return pos>(slidesTarget - slidesPositionTolerance) && pos<(slidesTarget + slidesPositionTolerance);
        };
    }

    public Action anglePitch(int position) {
        return telemetryPacket -> {
            pitchTarget = position;
            pitchControl.setSetpoint(pitchTarget);
            double feedforward2 = kG2 * (slidesMotor.getCurrentPosition() * ticksToInches) + 0;
            double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
            pitchMotor.setPower(feedforward3 + pid + feedforward2);

            double pos = pitchMotor.getCurrentPosition();

            return pos>(pitchTarget - pitchPositionTolerance) && pos<(pitchTarget + pitchPositionTolerance);
        };
    }

    public Action anglePitch(PitchPosition position) {
        return telemetryPacket -> {
            switch (position) {
                case HIGH:
                    pitchTarget = pitchHighPosition;
                    break;
                case MID:
                    pitchTarget = pitchMidPosition;
                    break;
                case DOWN:
                    pitchTarget = pitchLowPosition;
                    break;
            }
            pitchControl.setSetpoint(pitchTarget);
            double feedforward2 = kG2 * (slidesMotor.getCurrentPosition() * ticksToInches) + 0;
            double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
            double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
            pitchMotor.setPower(feedforward3 + pid + feedforward2);

            double pos = pitchMotor.getCurrentPosition();

            return pos>(pitchTarget - pitchPositionTolerance) && pos<(pitchTarget + pitchPositionTolerance);
        };
    }

    @Override
    public void update() {
    }

    public void slidesManual(double value) {
            slidesMotor.setPower(value);
    }

}