package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.PIDController;

@Config
public class Deposit implements Subsystem {

    /*
     * Servos
     */
    public ServoImplEx flipRight;
    public ServoImplEx flipLeft;
    public ServoImplEx extendoRight;
    public ServoImplEx extendoLeft;
    public ServoImplEx v4bServo;
    public Servo claw;

    private DepositState currentState = DepositState.NEUTRAL;

    public enum DepositState {
        INTAKE,
        CLOSE,
        TRANSFER,
        EXTEND,
        SCORE,
        NEUTRAL
    }

    public Deposit(HardwareMap hardwareMap) {

        flipRight = hardwareMap.get(ServoImplEx.class, "depositFlipRight");
        flipLeft = hardwareMap.get(ServoImplEx.class, "depositFlipLeft");

        extendoRight = hardwareMap.get(ServoImplEx.class, "extendoRight");
        extendoLeft = hardwareMap.get(ServoImplEx.class, "extendoLeft");

        v4bServo = hardwareMap.get(ServoImplEx.class, "depositV4bServo");

        claw = hardwareMap.get(Servo.class, "claw");
    }

    public void advanceState () {
        switch (currentState) {
            case NEUTRAL:
                currentState =  DepositState.INTAKE;
                v4bServo.setPosition(0.06);

                flipLeft.setPosition(0.65);
                flipRight.setPosition(0.65);

                extendoLeft.setPosition(0.48);
                extendoRight.setPosition(0.48);

                claw.setPosition(0.6);
                break;
            case INTAKE:
                currentState = DepositState.CLOSE;
                claw.setPosition(0.91);
                break;
            case CLOSE:
                currentState = DepositState.EXTEND;

                flipLeft.setPosition(0.05);
                flipRight.setPosition(0.05);

                extendoLeft.setPosition(0.95);
                extendoRight.setPosition(0.95);

                v4bServo.setPosition(.5);

                break;
            case EXTEND:
                currentState = DepositState.SCORE;
                flipLeft.setPosition(0.18);
                flipRight.setPosition(0.18);
                v4bServo.setPosition(.35);
                claw.setPosition(0.82);
                break;
            case SCORE:
                currentState = DepositState.NEUTRAL;
                break;
        }
    }

    public void rewindState () {
        switch (currentState) {
            case NEUTRAL:
                currentState = DepositState.SCORE;
                flipLeft.setPosition(0.18);
                flipRight.setPosition(0.18);
                v4bServo.setPosition(.35);
                claw.setPosition(0.81);
                break;
            case INTAKE:
                currentState = DepositState.NEUTRAL;
                break;
            case CLOSE:
                currentState = DepositState.INTAKE;
                v4bServo.setPosition(0.06);

                flipLeft.setPosition(0.65);
                flipRight.setPosition(0.65);

                extendoLeft.setPosition(0.48);
                extendoRight.setPosition(0.48);

                claw.setPosition(0.6);
                break;
            case EXTEND:
                currentState = DepositState.CLOSE;
                claw.setPosition(0.91);
                break;
            case SCORE:
                currentState = DepositState.EXTEND;

                flipLeft.setPosition(0.05);
                flipRight.setPosition(0.05);

                extendoLeft.setPosition(0.95);
                extendoRight.setPosition(0.95);

                v4bServo.setPosition(.5);
                break;
        }
    }

    @Override
    public void update() {
        switch (currentState) {

        }

    }

}