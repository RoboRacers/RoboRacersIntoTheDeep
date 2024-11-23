package org.firstinspires.ftc.teamcode.teleop.test;



import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.Deposit;
import org.firstinspires.ftc.teamcode.teleop.Rolling;


@TeleOp(name = "Pitch Test", group = "Test")
public class DepositTest extends LinearOpMode {
    public DcMotorImplEx pitchMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");

        while (opModeInInit()) {
            // 175
            // 420
        }

        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStopRequested()) {

            if (gamepad1.triangle) {
                pitchMotor.setTargetPosition(420);
                pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pitchMotor.setPower(0.75);
            } else if (gamepad1.cross) {
                pitchMotor.setTargetPosition(175);
                pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pitchMotor.setPower(0.75);
            } else if (gamepad1.circle) {
                pitchMotor.setTargetPosition(30);
                pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pitchMotor.setPower(0.75);
            } else {
                pitchMotor.setPower(gamepad1.left_stick_y);
            }

            telemetry.addData("Pitch Motor Position", pitchMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", pitchMotor.getPower());
            telemetry.addData("Pitch Current", pitchMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();

        }
    }
}
