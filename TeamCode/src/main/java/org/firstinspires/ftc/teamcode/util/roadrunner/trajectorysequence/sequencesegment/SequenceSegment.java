package org.firstinspires.ftc.teamcode.util.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;

import com.acmerobotics.roadrunner.TrajectoryBuilder;

import java.util.List;

public abstract class SequenceSegment {
    private final double duration;
    private final Pose2d startPose;
    private final Pose2d endPose;
    private final List<TrajectoryBuilder> markers;

    protected SequenceSegment(
            double duration,
            Pose2d startPose, Pose2d endPose,
            List<TrajectoryBuilder> markers
    ) {
        this.duration = duration;
        this.startPose = startPose;
        this.endPose = endPose;
        this.markers = markers;
    }

    public double getDuration() {
        return this.duration;
    }

    public Pose2d getStartPose() {
        return startPose;
    }

    public Pose2d getEndPose() {
        return endPose;
    }

    public List<TrajectoryBuilder> getMarkers() {
        return markers;
    }
}
