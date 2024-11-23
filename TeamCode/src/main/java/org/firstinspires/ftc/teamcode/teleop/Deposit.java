package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Deposit implements org.firstinspires.ftc.teamcode.robot.teleop.Subsystem {
    public ServoImplEx flipRightDeposit;
    public ServoImplEx flipLeftDeposit;
    public ServoImplEx pitch;
    public ServoImplEx claw;

    public DcMotorImplEx slidesRight;
    public DcMotorImplEx slidesLeft;

    public PIDController slidesPID = new PIDController(0.15, 0.1, 0.25);

    public Deposit(HardwareMap hardwareMap){
        flipRightDeposit = hardwareMap.get(ServoImplEx.class, "Flip_Right_Deposit");
        flipLeftDeposit = hardwareMap.get(ServoImplEx.class, "Flip_Left_Deposit");
        pitch = hardwareMap.get(ServoImplEx.class, "Pitch");
        claw = hardwareMap.get(ServoImplEx.class, "Claw");

        slidesRight = hardwareMap.get(DcMotorImplEx.class, "Slides_Right");
        slidesLeft = hardwareMap.get(DcMotorImplEx.class, "Slides_Left");

        slidesLeft.setDirection(DcMotorImplEx.Direction.REVERSE);
    }


    public void openClaw(){
        claw.setPosition(0.7);
    }

    public void closeClaw(){
        claw.setPosition(0.45);
    }

    public void goToGrab(){
        flipRightDeposit.setPosition(0.87);
        flipLeftDeposit.setPosition(0.87);
        pitch.setPosition(0.25);
    }
    public void goToRelease(){
        flipRightDeposit.setPosition(0);
        flipLeftDeposit.setPosition(0);
        pitch.setPosition(0.28);
    }

    public void setSlidePower(double power){
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
//        telemetry.addData("Claw", claw.getPosition());
//        telemetry.addData("Deposit FlipLeft", flipLeftDeposit.getPosition());
//        telemetry.addData("Deposit Pitch", pitch.getPosition());
//        telemetry.addData("Deposit SlidesLeft Position", slidesLeft.getCurrentPosition());
//        telemetry.addData("Deposit SlideRight Position", slidesRight.getCurrentPosition());
//        telemetry.addData("Deposit SlideLeft Power", slidesLeft.getPower());
//        telemetry.addData("Deposit SlideRight Power", slidesRight.getPower());
//        telemetry.update();
    }
}