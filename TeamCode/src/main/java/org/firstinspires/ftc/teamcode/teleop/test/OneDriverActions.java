package org.firstinspires.ftc.teamcode.teleop.test;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.PIDController;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

//@TeleOp(name = "LM2 One Driver", group = "Test")
public class OneDriverActions implements Subsystem {
    //Pitch Stuff

    Servo uno;
    Servo dos;

    public static double flipPos = 0;

        public  double kG = 0.027;
        public DcMotorImplEx slidesMotor;
        public DcMotorImplEx pitchMotor;
        public double kP2 = 0.005;
        public double kI2 = 0; // slides constant
        public double kD2 = 0.0008;
        public  double offset = 40;
        PIDController slidesControl;

        public  double target = 0; //slides
        public  double ticksPerMaxExtend = 1936;
        public  double ticksPerRightAngle = 930;

        public void Slides(HardwareMap hardwareMap) {
            slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
            pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
            uno = hardwareMap.get(Servo.class, "flipLeft");
            dos = hardwareMap.get(Servo.class, "flipRight");
            pitchMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
            pitchMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
            slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }



        public Action highBasket(){

            slidesControl.setCoefficients(kP2, kI2, kD2);
            slidesControl = new PIDController(kP2, kI2, kD2);
            final double ticksToInches = (double) 26 /ticksPerMaxExtend;
            final double ticksToDegrees = (double) 90 /ticksPerRightAngle;
            slidesControl.setSetpoint(target);
            return new Action() {


            }
                double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
                double pid2 = slidesControl.calculate(slidesMotor.getCurrentPosition());
                    slidesMotor.setPower(-(pid2 + feedforward));

        }
        public class flipUp implements Action{
            private boolean initialized = false;

            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    flipPos = 0.8;
                    initialized = true;
                    uno.setPosition(flipPos);
                    dos.setPosition(flipPos*0.94);
                }
                double pos = uno.getPosition();
                return pos==flipPos;
            }

        }
        public Action spinUp() {
            return new flipUp();
        }
    public class flipDown implements Action{
        private boolean initialized = false;

        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                flipPos = 0.25;
                initialized = true;
                uno.setPosition(flipPos);
                dos.setPosition(flipPos*0.94);
            }
            double pos = uno.getPosition();
            return pos==flipPos;
        }

    }
    public Action spinDown() {
        return new flipDown();
    }
    public class slidesUp implements Action{
        private boolean initialized = false;

        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                target = 1650;
                slidesControl.setCoefficients(kP2, kI2, kD2);
                slidesControl = new PIDController(kP2, kI2, kD2);
                final double ticksToInches = (double) 26 /ticksPerMaxExtend;
                final double ticksToDegrees = (double) 90 /ticksPerRightAngle;
                slidesControl.setSetpoint(target);
                double feedforward = kG * Math.sin(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
                double pid2 = slidesControl.calculate(slidesMotor.getCurrentPosition());
                slidesMotor.setPower(-(pid2 + feedforward));
                initialized = true;
            }

            double pos = slidesMotor.getCurrentPosition();
            return pos==flipPos;
        }

    }
    public Action extendSlide() {
        return new slidesUp();
    }
    
    @Override
    public void update() {

    }



}
