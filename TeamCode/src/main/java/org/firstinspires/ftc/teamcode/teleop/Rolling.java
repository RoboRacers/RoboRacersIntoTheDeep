package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Rolling implements org.firstinspires.ftc.teamcode.robot.teleop.Subsystem {
    DcMotor slideMotor = hardwareMap.get(DcMotor.class, "Horizontal_Slides");;
    ServoImplEx flipLeft = hardwareMap.get(ServoImplEx.class, "Flip_Left_Intake");;
    ServoImplEx flipRight = hardwareMap.get(ServoImplEx.class, "Flip_Right_Intake");;
    CRServoImplEx intakeMotor = hardwareMap.get(CRServoImplEx.class, "Servo_Intake");;


    public void setSlidePower(double power) {
        slideMotor.setPower(power);
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
    }
}