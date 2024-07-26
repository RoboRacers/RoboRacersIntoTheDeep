package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.LoggingUtil;

@TeleOp(name = "File Writer Test", group = "Test")
public class FileWriterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        LoggingUtil logger = new LoggingUtil("distance_sensor_logs", true);
        telemetry.addLine(logger.path);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            logger.
                    addData
                            (
                    distanceSensor.getDistance(DistanceUnit.INCH)
            );
            logger.update();
        }
        logger.close();
    }
}


