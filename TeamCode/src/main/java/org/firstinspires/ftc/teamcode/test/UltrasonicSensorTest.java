package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;


@TeleOp(name = "Ultrasonic Sensor Test", group = "Test")
public class UltrasonicSensorTest extends LinearOpMode {

    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        while (!isStopRequested()) {

            // Telemetry
            telemetry.addLine("Ultrasonic Testing");
            telemetry.addData("Left Ultrasonic", drive.leftUltrasonic.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Ultrasonic", drive.rightUltrasonic.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }
    }
}
