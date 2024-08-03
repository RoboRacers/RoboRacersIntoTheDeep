package org.firstinspires.ftc.teamcode.test;

import java.util.LinkedList;
import java.util.Queue;

public class WeightedMovingAverageEdited {
    private double currentAvg;
    private double weight;


    public WeightedMovingAverageEdited(double weight) {
       this.weight = weight;
       this.currentAvg = 0;
    }



    public double getAvg(double value) {
        if (this.currentAvg == 0) {
            this.currentAvg=value;
        }
        else {
            this.currentAvg = this.currentAvg*(1-this.weight)+value*this.weight;
        }
        return this.currentAvg;
    }
}
