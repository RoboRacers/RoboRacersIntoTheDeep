package org.firstinspires.ftc.teamcode.actions.newactions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.actions.Action;

public class example {
    DcMotorImplEx motorRight;
    DcMotorImplEx motorLeft;

    public example(HardwareMap hardwareMap){
        motorRight = hardwareMap.get(DcMotorImplEx.class, "Right_Slides");
        motorLeft = hardwareMap.get(DcMotorImplEx.class, "Left_Slides");
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Action init(){
        return new Action() {
            private boolean init = false;
            @Override
            public boolean run(TelemetryPacket p) {
                if(!init){
                    motorRight.setPower(0.5);
                    motorLeft.setPower(0.5);
                    motorRight.setTargetPosition(0);
                    motorLeft.setTargetPosition(0);
                    init = true;
                }
                motorRight.setPower(0);
                motorLeft.setPower(0);
                return false;
            }
        };
    }

    public Action up(){
        return new Action() {
            private boolean init = false;
            @Override
            public boolean run(TelemetryPacket p) {
                if(!init){
                    motorRight.setPower(0.5);
                    motorLeft.setPower(0.5);
                    motorRight.setTargetPosition(1000);
                    motorLeft.setTargetPosition(1000);
                    init = true;
                }
                return motorRight.isBusy() && motorLeft.isBusy();
            }
        };
    }


}