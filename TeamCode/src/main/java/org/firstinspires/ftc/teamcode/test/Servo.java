package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;



public class Servo extends LinearOpMode {

	public Servo() {
		requires(servo);
	}

	
	protected void initialize() {
	}


	protected void execute() {
		servo.setLeftAngle(oi.auto.getLeftStickRaw_X() * 180);
		servo.setRightAngle(oi.auto.getLeftStickRaw_X() * 180);
	}

	
	protected boolean isFinished() {
		return false;
	}

	
	protected void end() {
	}

	
	protected void interrupted() {
	}
}
