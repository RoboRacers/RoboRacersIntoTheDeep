package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Deposit implements Subsystem{
    ServoImplEx flipRight = hardwareMap.get(ServoImplEx.class, "Flip_Right");
    ServoImplEx flipLeft = hardwareMap.get(ServoImplEx.class, "Flip_Left");
    ServoImplEx pitch = hardwareMap.get(ServoImplEx.class, "Pitch");
    ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "Claw");

    DcMotorImplEx slidesRight = hardwareMap.get(DcMotorImplEx.class, "slides_Right");
    DcMotorImplEx slidesLeft = hardwareMap.get(DcMotorImplEx.class, "slides_Left");

    PIDController slidesPID = new PIDController(1, 1, 1);

    public void openClaw(){
        claw.setPosition(1);
    }
    public void closeClaw(){
        claw.setPosition(0);
    }

    public void goToGrab(){
        flipRight.setPosition(1);
        flipLeft.setPosition(1);
        pitch.setPosition(1);
    }
    public void goToRelease(){
        flipRight.setPosition(0);
        flipLeft.setPosition(0);
        pitch.setPosition(0);
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

    }

}
