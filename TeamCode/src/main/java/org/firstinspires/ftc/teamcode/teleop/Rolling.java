package org.firstinspires.ftc.teamcode.robot.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Rolling implements Subsystem {
    private DcMotorImplEx slideMotor;
    private ServoImplEx flipLeft;
    private ServoImplEx flipRight;
    private CRServoImplEx intakeMotor;

    public Rolling() {
        slideMotor = hardwareMap.get(DcMotorImplEx.class, "Horizontal_Slides");
        flipLeft = hardwareMap.get(ServoImplEx.class, "Flip_Left_Intake");
        flipRight = hardwareMap.get(ServoImplEx.class, "Flip_Right_Intake");
        intakeMotor = hardwareMap.get(CRServoImplEx.class, "Servo_Intake");

    }

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