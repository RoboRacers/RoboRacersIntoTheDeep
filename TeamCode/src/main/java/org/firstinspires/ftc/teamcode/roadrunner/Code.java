package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;

@TeleOp(name = "pai", group = "0000-Final")
public class Code extends LinearOpMode {
    DcMotorImplEx leftFront;
    DcMotorImplEx rightFront;
    DcMotorImplEx leftBack;


    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorImplEx.class, "zero");//dr4b
        rightFront = hardwareMap.get(DcMotorImplEx.class, "one");//dr4b
        leftBack = hardwareMap.get(DcMotorImplEx.class, "two");//slides motor





        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        while (opModeInInit()) {
        }

        waitForStart();

        while (!isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad1.a) {
              leftFront.setPower(1);
            }
            else{
                leftFront.setPower(0);
            }

             if (gamepad1.x) {

                 leftBack.setPower(1);

            }
             else{
                 leftBack.setPower(0);
             }if (gamepad1.y) {
                rightFront.setPower(1);

            }
            else{
                rightFront.setPower(0);
            }

            telemetry.update();

        }

    }
}
