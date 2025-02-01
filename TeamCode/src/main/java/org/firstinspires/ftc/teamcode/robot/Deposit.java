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
    ServoImplEx flipRight;
    ServoImplEx flipLeft;
    ServoImplEx extendoRight;
    ServoImplEx extendoLeft;
    ServoImplEx v4bServo;
    Servo claw;

    private DepositState currentState;

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
                break;
            case INTAKE:
                currentState = DepositState.CLOSE;
                break;
            case CLOSE:
                currentState = DepositState.EXTEND;
                flipLeft.setPosition(0.1);
                flipRight.setPosition(0.1);
                v4bServo.setPosition(.8);
                break;
            case EXTEND:
                currentState = DepositState.SCORE;
                flipLeft.setPosition(0.3);
                flipRight.setPosition(0.3);
                v4bServo.setPosition(.8);
                break;
            case SCORE:
                currentState = DepositState.NEUTRAL;
                break;
        }
    }

    public void rewindState () {
        switch (currentState) {
            case NEUTRAL:
                currentState =  DepositState.SCORE;
                break;
            case INTAKE:
                currentState = DepositState.NEUTRAL;
                break;
            case CLOSE:
                currentState = DepositState.INTAKE;
                break;
            case EXTEND:
                currentState = DepositState.CLOSE;
                break;
            case SCORE:
                currentState = DepositState.EXTEND;
                break;
        }
    }

    @Override
    public void update() {
        switch (currentState) {

        }

    }

}