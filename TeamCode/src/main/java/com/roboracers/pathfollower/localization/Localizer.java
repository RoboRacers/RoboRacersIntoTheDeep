package com.roboracers.pathfollower.localization;

import com.roboracers.pathfollower.geometry.Pose2d;

public interface Localizer {

    /**
     * Current robot pose estimate.
     */
    Pose2d getPoseEstimate();


    void setPoseEstimate(Pose2d pose);

    /**
     * Current robot pose velocity (optional)
     */
    Pose2d getPoseVelocity();

    /**
     * Completes a single localization update.
     */
    void update();
}
