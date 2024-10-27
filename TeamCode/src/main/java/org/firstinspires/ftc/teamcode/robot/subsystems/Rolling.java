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
    public void intakeFlip(){

    }
    @Override
    public void update() {
        // Update logic if needed
    }
}
