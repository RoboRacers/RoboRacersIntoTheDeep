package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Rolling implements org.firstinspires.ftc.teamcode.robot.teleop.Subsystem {
    public DcMotor slideMotor = hardwareMap.get(DcMotor.class, "Horizontal_Slides");;
    public ServoImplEx flipLeft = hardwareMap.get(ServoImplEx.class, "Flip_Left_Intake");;
    public ServoImplEx flipRight = hardwareMap.get(ServoImplEx.class, "Flip_Right_Intake");;
    public CRServoImplEx intakeMotor = hardwareMap.get(CRServoImplEx.class, "Servo_Intake");;
    public PIDController intakePID = new PIDController(0.1, 0.01, 0.05);



    public void setSlidePower(double power) {
        slideMotor.setPower(power);
    }
    public void setSlidePos(double position){
        intakePID.setSetpoint(position);
        double output = intakePID.calculate(slideMotor.getCurrentPosition());
        setSlidePower(output);
    }
    //Dynamic Intake rotation
    public void set(double position){
        flipLeft.setPosition(position);
        flipRight.setPosition(position);
    }

    public void setIntake(){
        intakeMotor.setPower(1);
    }

    public void setOutake(){
        intakeMotor.setPower(-1);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
    }

    // Preset Intake rotations
    public void setIntakeUp(){
        flipRight.setPosition(0.8);
        flipLeft.setPosition(0.8);
    }
    public void setIntakeDown(){
        flipLeft.setPosition(0.2);
        flipRight.setPosition(0.2);
    }
    @Override
    public void update() {
        // Update logic if needed
        telemetry.addData("Rolling Intake Power", intakeMotor.getPower());
        telemetry.addData("Intake Flip Left", flipLeft.getPosition());
        telemetry.addData("Intake Flip Right", flipRight.getPosition());
        telemetry.addData("Horizontal Slides Power", slideMotor.getPower());
        telemetry.addData("Horizontal Slides Position", slideMotor.getCurrentPosition());
        telemetry.update();
    }
}