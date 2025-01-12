package org.firstinspires.ftc.teamcode.auton;



import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Assembly;

//import org.firstinspires.ftc.teamcode.teleop.Deposit;
//import org.firstinspires.ftc.teamcode.teleop.PIDController;
//import org.firstinspires.ftc.teamcode.teleop.Rolling;


@Autonomous(name = "Bet", group = "16481-IntoTheDeep")
public class oneplusfour extends LinearOpMode {

    Assembly assembly;


    MecanumDrive drive;


    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        assembly = new Assembly(hardwareMap);

        runtime.reset();

        Action traj = drive.actionBuilder(new Pose2d(0,0,0))
                // Init Position
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {assembly.claw.setPosition(0.35);
                            return false;
                        },
                        telemetryPacket -> {assembly.claw.setPosition(0.35);
                            return false;
                        },
                        telemetryPacket -> {
                            assembly.flipLeft.setPosition(0.130);
                            assembly.flipRight.setPosition(0.130 * 0.94);
                            return false;
                        },
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_MID_POSITION);
                            return false;
                        }
                        //                        new SleepAction(2)
                ))
                .splineToLinearHeading(new Pose2d(7,36,Math.toRadians(-45)),Math.toRadians(-45))

//           above or this one     .splineToLinearHeading(new Pose2d(11,34,Math.toRadians(0)),Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        // To score Preload in first basket
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_HIGH_POSITION);
                            return false;
                        },
                        new SleepAction(1),
                        telemetryPacket -> {assembly.slideTarget = Assembly.SLIDES_HIGH_POSITION;
                            return false;
                        },
                        new SleepAction(1),
                        telemetryPacket -> {
                            assembly.flipLeft.setPosition(0.5);
                            assembly.flipRight.setPosition(0.5 * 0.94);
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {assembly.claw.setPosition(0.1);
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {
                            assembly.flipLeft.setPosition(0.130);
                            assembly.flipRight.setPosition(0.130 * 0.94);
                            return false;
                        },
                        new SleepAction(1),
                        telemetryPacket -> {assembly.slideTarget = Assembly.SLIDES_LOW_POSITION;
                            return false;
                        },
                        new SleepAction(1),
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_LOW_POSITION);
                            return false;
                        }                ))
                // To pick up 1st sample(right side) on floor
                .strafeToLinearHeading(new Vector2d(12, 30.5), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(12, 30), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_LOW_POSITION);
                            return false;
                        },
                        telemetryPacket -> {assembly.slideTarget = Assembly.SLIDES_MID_POSITION;
                            return false;
                        },
                        new SleepAction(1),
                        telemetryPacket -> {assembly.claw.setPosition(0.35);
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_HIGH_POSITION);
                            return false;
                        }
                ))
                // To drop 1st sample from floor into high basket
                .strafeToLinearHeading(new Vector2d(7, 36), Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {assembly.slideTarget = Assembly.SLIDES_LOW_POSITION;
                            return false;
                        },
                        elemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_HIGH_POSITION);
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {assembly.slideTarget = Assembly.SLIDES_HIGH_POSITION;
                            return false;
                        },
                        new SleepAction(1),
                        assembly.flipMid(),
                        new SleepAction(0.5),
                        telemetryPacket -> {assembly.claw.setPosition(0.35);
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {
                            assembly.flipLeft.setPosition(0.130);
                            assembly.flipRight.setPosition(0.130 * 0.94);
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_LOW_POSITION);
                            return false;
                        }
                ))
                // To pickup 2nd sample(middle) from the floor
                .strafeToLinearHeading(new Vector2d(14, 40), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(12.5, 40), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_MID_POSITION);
                            return false;
                        },
                        telemetryPacket -> {
                            assembly.flipLeft.setPosition(0.130);
                            assembly.flipRight.setPosition(0.130 * 0.94);
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_LOW_POSITION);
                            return false;
                        },
                        new SleepAction(1),
                        telemetryPacket -> {assembly.claw.setPosition(0.35);
                            return false;
                        },
                        new SleepAction(1),
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_HIGH_POSITION);
                            return false;
                        }
                ))
                // To drop 2nd sample into high basket
                .strafeToLinearHeading(new Vector2d(8, 36), Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {assembly.setSlideTarget(Assembly.SLIDES_LOW_POSITION);
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_HIGH_POSITION);
                            return false;
                        },
                        new SleepAction(1),
                        telemetryPacket -> {assembly.setSlideTarget(Assembly.SLIDES_HIGH_POSITION);
                            return false;
                        },
                        new SleepAction(1),
                        telemetryPacket -> {
                            assembly.flipLeft.setPosition(0.5);
                            assembly.flipRight.setPosition(0.5 * 0.94);
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {assembly.claw.setPosition(0.35);
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {
                            assembly.flipLeft.setPosition(0.130);
                            assembly.flipRight.setPosition(0.130 * 0.94);
                            return false;
                        },
                        new SleepAction(1),
                        telemetryPacket -> {assembly.setSlideTarget(Assembly.SLIDES_LOW_POSITION);
                            return false;
                        },
                        new SleepAction(0.5),
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_LOW_POSITION);
                            return false;
                        },
                        new SleepAction(0.5)
                ))
                // To pickup 3rd sample(left) from the floor
                .strafeToLinearHeading(new Vector2d(35.5, 30), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(35.5, 32.5), Math.toRadians(90))
                .stopAndAdd(new SequentialAction(
                        //assembly.extendSlide(475),
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_LOW_POSITION);
                            return false;
                        },
                        assembly.flipCus(0.2),
                        telemetryPacket -> {assembly.claw.setPosition(0.35);
                            return false;
                        },
                        assembly.rotateClaw(0.91),
                        new SleepAction(0.5),
                        new SleepAction(0.5),
                        assembly.flipCus(0.3),
                        telemetryPacket -> {assembly.claw.setPosition(0.35);
                            return false;
                        },                        new SleepAction(1000),
                        new SleepAction(1),
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_MID_POSITION);
                            return false;
                        }
                ))
                // To drop 3rd sample into high basket
                .strafeToLinearHeading(new Vector2d(11, 35), Math.toRadians(-45))
                .strafeToLinearHeading(new Vector2d(8, 36), Math.toRadians(-45))
                .stopAndAdd(new SequentialAction(
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_HIGH_POSITION);
                            return false;
                        },
                        new SleepAction(1),
                        new SleepAction(0.5),
                        telemetryPacket -> {assembly.setSlideTarget(Assembly.SLIDES_HIGH_POSITION);
                            return false;
                        },
                        new SleepAction(1),
                        assembly.flipMid(),
                        new SleepAction(1.5),
                        new SleepAction(0.5),
                        telemetryPacket -> {assembly.claw.setPosition(0.35);
                            return false;
                        },
                        new SleepAction(1),
                        telemetryPacket -> {
                            assembly.flipLeft.setPosition(0.130);
                            assembly.flipRight.setPosition(0.130 * 0.94);
                            return false;
                        },

                        new SleepAction(1),
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.SLIDES_LOW_POSITION);
                            return false;
                        },
                        new SleepAction(2),
                        telemetryPacket -> {assembly.setPitchTarget(Assembly.PITCH_LOW_POSITION);
                            return false;
                        },
                        new SleepAction(3)

                ))

                .build();
        while (opModeInInit()){
            assembly.claw.setPosition(0.35);
            assembly.flipMid();
            assembly.setPitchTarget(Assembly.PITCH_LOW_POSITION);
            assembly.setSlideTarget(Assembly.SLIDES_LOW_POSITION);
            //                    new SleepAction(2);
            //                    new SleepAction(2);
        }


        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                traj
                        ),
                        telemetryPacket -> {
                            assembly.update();
                            // telemtry here
                            telemetry.update();
                            return opModeIsActive();
                        }
                )
        );

    }
}