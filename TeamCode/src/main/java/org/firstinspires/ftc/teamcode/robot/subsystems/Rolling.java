package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Rolling implements Subsystem {
    private DcMotorImplEx intakeMotor;
    private ServoImplEx flipLeft;
    private ServoImplEx flipRight;



    public Rolling(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "IntakeMotor");
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }
    //Dynamic Intake rotation
    public void setIntakePosition(double position){
        flipLeft.setPosition(position);
        flipRight.setPosition(position);
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
