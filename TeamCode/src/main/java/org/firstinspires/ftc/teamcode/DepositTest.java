package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.useless.TankDrive;
@TeleOp(name ="Deposit", group = "16481-IntoTheDeep")
public class DepositTest extends LinearOpMode {
    public DcMotorImplEx intakeMotor;
    public ServoImplEx flipLeft;

    public CRServoImplEx rolling;

    public ServoImplEx flipRight;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            flipLeft = hardwareMap.get(ServoImplEx.class, "flipLeft");
            flipRight = hardwareMap.get(ServoImplEx.class, "flipRight");
            intakeMotor = hardwareMap.get(DcMotorImplEx.class,"intakeRoll");
            rolling = hardwareMap.get(CRServoImplEx.class,"rolling");

            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.a){
                    flipLeft.setPosition(0.7);
                }
                else if(gamepad1.b){
                    flipRight.setPosition(0.3);
                }
                else if(gamepad1.right_trigger>0.1){
                    rolling.setPower(0.5);
                }

                intakeMotor.setPower(gamepad1.right_stick_y);



                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
