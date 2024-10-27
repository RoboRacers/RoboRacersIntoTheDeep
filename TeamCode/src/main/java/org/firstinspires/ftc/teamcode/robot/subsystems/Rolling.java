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
    // The position values below are test values i.e. modify these to be the actual position values.
    public void setIntakeUp(){
        flipRight.setPosition(1);
        flipLeft.setPosition(1);
    }
    public void setIntakeDown(){
        flipLeft.setPosition(0);
        flipRight.setPosition(0);
    }
    @Override
    public void update() {
        // Update logic if needed
    }
}
