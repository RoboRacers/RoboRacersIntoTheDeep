package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.subsystems.Vision;
import org.firstinspires.ftc.teamcode.modules.util.LoggingUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "File Writer Test", group = "Test")
public class FileWriterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        LoggingUtil logger = new LoggingUtil("test_log", true);

        logger.addData(1001);
        logger.addData("test");
        logger.
    }
}


