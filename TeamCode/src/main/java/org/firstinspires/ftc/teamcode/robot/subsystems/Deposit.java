package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Deposit implements Subsystem{
    ServoImplEx flipRight = hardwareMap.get(ServoImplEx.class, "Flip_Right_Deposit");
    ServoImplEx flipLeft = hardwareMap.get(ServoImplEx.class, "Flip_Left_Deposit");
    ServoImplEx pitch = hardwareMap.get(ServoImplEx.class, "Pitch");
    ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "Claw");

    DcMotorImplEx slidesRight = hardwareMap.get(DcMotorImplEx.class, "Slides_Right");
    DcMotorImplEx slidesLeft = hardwareMap.get(DcMotorImplEx.class, "Slides_Left");

    PIDController slidesPID = new PIDController(0.1, 0.1, 0.1);

    public void openClaw(){
        claw.setPosition(0);
    }
    public void closeClaw(){
        claw.setPosition(0.7);
    }

    public void goToGrab(){
        flipRight.setPosition(1);
        flipLeft.setPosition(1);
        pitch.setPosition(0.228);
    }
    public void goToRelease(){
        flipRight.setPosition(0);
        flipLeft.setPosition(0);
        pitch.setPosition(0.28);
    }

    private void setSlidePower(double power){
        slidesRight.setPower(power);
        slidesLeft.setPower(power);
    }

    public void setSlidePos(double position){
        slidesPID.setSetpoint(position);
        double output = slidesPID.calculate((slidesRight.getCurrentPosition() + slidesLeft.getCurrentPosition())/2);
        setSlidePower(output);
    }

    @Override
    public void update() {
        telemetry.addData("Shreesh is ", "bum");

    }
}