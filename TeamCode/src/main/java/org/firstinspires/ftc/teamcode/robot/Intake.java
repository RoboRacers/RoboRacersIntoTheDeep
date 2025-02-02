package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Intake implements Subsystem {

    /*
     * Servos
     */
    ServoImplEx flipRight;
    ServoImplEx flipLeft;
    public DcMotorImplEx intakeMotor;
    public DcMotorImplEx slidesMotor;
    ServoImplEx v4bServo;


    private DepositState currentState = DepositState.NEUTRAL;

    public enum DepositState {
        INTAKE,
        CLOSE,
        TRANSFER,
        EXTEND,
        SCORE,
        NEUTRAL
    }

    public Intake(HardwareMap hardwareMap) {

        flipRight = hardwareMap.get(ServoImplEx.class, "depositFlipRight");
        flipLeft = hardwareMap.get(ServoImplEx.class, "depositFlipLeft");

        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");

        v4bServo = hardwareMap.get(ServoImplEx.class, "depositV4bServo");

        slidesMotor.setDirection(DcMotorImplEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorImplEx.Direction.REVERSE);
    }

    public void advanceState () {
        switch (currentState) {
            case NEUTRAL:
                currentState =  DepositState.INTAKE;
                v4bServo.setPosition(0.4);
                flipLeft.setPosition(0.2-0.01);
                flipRight.setPosition(0.2);
                break;
            case INTAKE:
                currentState = DepositState.NEUTRAL;
                v4bServo.setPosition(0.55);
                flipLeft.setPosition(0.50-0.01);
                flipRight.setPosition(0.50);
                break;
        }
    }
    public void intakeForward () {
        intakeMotor.setPower(0.8);
    }
    public void intakeBack () {
        intakeMotor.setPower(-0.6);

    }
    public void intakeStop () {
        intakeMotor.setPower(0);

    }
    public void slideExtend () {
        slidesMotor.setPower(0.8);

    }
    public void slideRetract () {
        slidesMotor.setPower(-0.5);

    }

    public void slidesStop () {
        slidesMotor.setPower(0);

    }
//    public void rewindState () {
//        switch (currentState) {
//            case NEUTRAL:
//                currentState =  DepositState.SCORE;
//                break;
//            case INTAKE:
//                currentState = DepositState.NEUTRAL;
//                break;
//            case CLOSE:
//                currentState = DepositState.INTAKE;
//                break;
//            case EXTEND:
//                currentState = DepositState.CLOSE;
//                break;
//            case SCORE:
//                currentState = DepositState.EXTEND;
//                break;
//        }
//    }

    @Override
    public void update() {
        switch (currentState) {

        }

    }

}