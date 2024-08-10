package org.firstinspires.ftc.teamcode.pathfollowertest;

import com.roboracers.topgear.geometry.Vector2d;

public class fieldCentricToRobotCentricTest {
    public static void main(String[] args) {
        Vector2d unitVec = new Vector2d(1,1);

        System.out.println(unitVec.fieldToRobotCentric(Math.toRadians(90)).toString());
    }
}
