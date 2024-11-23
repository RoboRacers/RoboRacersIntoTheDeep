package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Rolling implements org.firstinspires.ftc.teamcode.robot.teleop.Subsystem {
    public DcMotor slideMotor;
    public ServoImplEx flipLeftIntake;
    public ServoImplEx flipRightIntake;
//    public CRServoImplEx intakeMotor;

    public PIDController intakePID = new PIDController(0.15, 0.05, 0.2);

    public Rolling(HardwareMap hardwareMap){
        slideMotor = hardwareMap.get(DcMotor.class, "Horizontal_Slides");
        flipLeftIntake = hardwareMap.get(ServoImplEx.class, "Flip_Left_Intake");
        flipRightIntake = hardwareMap.get(ServoImplEx.class, "Flip_Right_Intake");
//        intakeMotor = hardwareMap.get(CRServoImplEx.class, "Servo_Intake");
    }



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
        flipLeftIntake.setPosition(position);
        flipRightIntake.setPosition(position);
    }

//    public void setIntake(){
//        intakeMotor.setPower(0.8);
//    }

//    public void setOutake(){
//        intakeMotor.setPower(-0.8);
//    }

//    public void stopIntake(){
//        intakeMotor.setPower(0);
//    }

    // Preset Intake rotations
    public void setIntakeUp(){
        flipRightIntake.setPosition(0.8);
        flipLeftIntake.setPosition(0.8);
    }
    public void setIntakeDown(){
        flipLeftIntake.setPosition(0.2);
        flipRightIntake.setPosition(0.2);
    }
    @Override
    public void update() {
        // Update logic if needed
//        telemetry.addData("Rolling Intake Power", intakeMotor.getPower());
//        telemetry.addData("Intake Flip Left", flipLeftIntake.getPosition());
//        telemetry.addData("Intake Flip Right", flipRightIntake.getPosition());
//        telemetry.addData("Horizontal Slides Power", slideMotor.getPower());
//        telemetry.addData("Horizontal Slides Position", slideMotor.getCurrentPosition());
//        telemetry.update();
    }
}