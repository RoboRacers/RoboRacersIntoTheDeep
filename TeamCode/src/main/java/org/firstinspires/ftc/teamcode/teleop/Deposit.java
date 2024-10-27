package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.teleop.PIDController;

public class Deposit implements org.firstinspires.ftc.teamcode.robot.teleop.Subsystem {
    public ServoImplEx flipRight = hardwareMap.get(ServoImplEx.class, "Flip_Right_Deposit");
    public ServoImplEx flipLeft = hardwareMap.get(ServoImplEx.class, "Flip_Left_Deposit");
    public ServoImplEx pitch = hardwareMap.get(ServoImplEx.class, "Pitch");
    public ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "Claw");

    public DcMotorImplEx slidesRight = hardwareMap.get(DcMotorImplEx.class, "Slides_Right");
    public DcMotorImplEx slidesLeft = hardwareMap.get(DcMotorImplEx.class, "Slides_Left");

    public PIDController slidesPID = new PIDController(0.1, 0.01, 0.05);


    public void openClaw(){
        claw.setPosition(.4);
    }

    public void closeClaw(){
        claw.setPosition(0.7);
    }

    public void goToGrab(){
        flipRight.setPosition(0.85);
        flipLeft.setPosition(0.85);
        pitch.setPosition(0.2);
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
        telemetry.addData("Claw", claw.getPosition());
        telemetry.addData("Deposit FlipLeft", flipLeft.getPosition());
        telemetry.addData("Deposit Pitch", pitch.getPosition());
        telemetry.addData("Deposit SlidesLeft Position", slidesLeft.getCurrentPosition());
        telemetry.addData("Deposit SlideRight Position", slidesRight.getCurrentPosition());
        telemetry.addData("Deposit SlideLeft Power", slidesLeft.getPower());
        telemetry.addData("Deposit SlideRight Power", slidesRight.getPower());
        telemetry.update();
    }
}