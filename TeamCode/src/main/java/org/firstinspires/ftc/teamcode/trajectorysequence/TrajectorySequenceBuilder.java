package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.List;

/**
 * Builder for creating trajectory sequences.
 * A trajectory sequence is a series of trajectory segments, turns, and waits
 * that can be executed as a single unit.
 */
public class TrajectorySequenceBuilder {
    // Current pose for starting new segments
    private Pose2d currentPose;
    
    // Default constraints for trajectory generation
    private final TrajectoryVelocityConstraint baseVelConstraint;
    private final TrajectoryAccelerationConstraint baseAccelConstraint;
    
    // Default constraints for turning
    private final double baseTurnConstraintMaxAngVel;
    private final double baseTurnConstraintMaxAngAccel;
    
    // List of segments being built
    private final List<TrajectorySequence.SequenceSegment> sequenceSegments;

    /**
     * Creates a new TrajectorySequenceBuilder
     * 
     * @param startPose the starting pose for the trajectory sequence
     * @param baseVelConstraint the default velocity constraint
     * @param baseAccelConstraint the default acceleration constraint
     * @param baseTurnConstraintMaxAngVel the default maximum angular velocity for turns
     * @param baseTurnConstraintMaxAngAccel the default maximum angular acceleration for turns
     */
    public TrajectorySequenceBuilder(
            Pose2d startPose,
            TrajectoryVelocityConstraint baseVelConstraint,
            TrajectoryAccelerationConstraint baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        this.currentPose = startPose;
        this.baseVelConstraint = baseVelConstraint;
        this.baseAccelConstraint = baseAccelConstraint;
        this.baseTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel;
        this.baseTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel;

        this.sequenceSegments = new ArrayList<>();
    }

    /**
     * Adds a line segment to the trajectory
     * 
     * @param endPosition the end position of the line
     * @return this builder
     */
    public TrajectorySequenceBuilder lineTo(Vector2d endPosition) {
        return addPath(() -> new TrajectoryBuilder(currentPose, baseVelConstraint, baseAccelConstraint)
                .lineTo(endPosition)
                .build()
        );
    }

    /**
     * Adds a line segment to the trajectory with custom constraints
     * 
     * @param endPosition the end position of the line
     * @param velConstraint the velocity constraint for this segment
     * @param accelConstraint the acceleration constraint for this segment
     * @return this builder
     */
    public TrajectorySequenceBuilder lineTo(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> new TrajectoryBuilder(currentPose, velConstraint, accelConstraint)
                .lineTo(endPosition)
                .build()
        );
    }

    /**
     * Adds a line segment to the trajectory with a specified heading
     * 
     * @param endPosition the end position of the line
     * @param heading the heading to maintain during the motion
     * @return this builder
     */
    public TrajectorySequenceBuilder lineToConstantHeading(Vector2d endPosition) {
        return addPath(() -> new TrajectoryBuilder(currentPose, baseVelConstraint, baseAccelConstraint)
                .lineToConstantHeading(endPosition)
                .build()
        );
    }

    /**
     * Adds a line segment to the trajectory with a specified heading and custom constraints
     * 
     * @param endPosition the end position of the line
     * @param velConstraint the velocity constraint for this segment
     * @param accelConstraint the acceleration constraint for this segment
     * @return this builder
     */
    public TrajectorySequenceBuilder lineToConstantHeading(
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> new TrajectoryBuilder(currentPose, velConstraint, accelConstraint)
                .lineToConstantHeading(endPosition)
                .build()
        );
    }

    /**
     * Adds a line segment that maintains position but changes heading
     * 
     * @param heading the target heading
     * @return this builder
     */
    public TrajectorySequenceBuilder lineToLinearHeading(Pose2d endPose) {
        return addPath(() -> new TrajectoryBuilder(currentPose, baseVelConstraint, baseAccelConstraint)
                .lineToLinearHeading(endPose)
                .build()
        );
    }

    /**
     * Adds a line segment that maintains position but changes heading with custom constraints
     * 
     * @param endPose the target pose
     * @param velConstraint the velocity constraint for this segment
     * @param accelConstraint the acceleration constraint for this segment
     * @return this builder
     */
    public TrajectorySequenceBuilder lineToLinearHeading(
            Pose2d endPose,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> new TrajectoryBuilder(currentPose, velConstraint, accelConstraint)
                .lineToLinearHeading(endPose)
                .build()
        );
    }

    /**
     * Adds a line segment to follow a spline path
     * 
     * @param endPosition the end position
     * @param endTangent the ending tangent direction
     * @return this builder
     */
    public TrajectorySequenceBuilder splineTo(Vector2d endPosition, double endTangent) {
        return addPath(() -> new TrajectoryBuilder(currentPose, baseVelConstraint, baseAccelConstraint)
                .splineTo(endPosition, endTangent)
                .build()
        );
    }

    /**
     * Adds a line segment to follow a spline path with custom constraints
     * 
     * @param endPosition the end position
     * @param endTangent the ending tangent direction
     * @param velConstraint the velocity constraint for this segment
     * @param accelConstraint the acceleration constraint for this segment
     * @return this builder
     */
    public TrajectorySequenceBuilder splineTo(
            Vector2d endPosition, double endTangent,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> new TrajectoryBuilder(currentPose, velConstraint, accelConstraint)
                .splineTo(endPosition, endTangent)
                .build()
        );
    }

    /**
     * Adds a spline segment with constant heading
     * 
     * @param endPosition the end position
     * @param endTangent the ending tangent direction
     * @return this builder
     */
    public TrajectorySequenceBuilder splineToConstantHeading(Vector2d endPosition, double endTangent) {
        return addPath(() -> new TrajectoryBuilder(currentPose, baseVelConstraint, baseAccelConstraint)
                .splineToConstantHeading(endPosition, endTangent)
                .build()
        );
    }

    /**
     * Adds a spline segment with constant heading and custom constraints
     * 
     * @param endPosition the end position
     * @param endTangent the ending tangent direction
     * @param velConstraint the velocity constraint for this segment
     * @param accelConstraint the acceleration constraint for this segment
     * @return this builder
     */
    public TrajectorySequenceBuilder splineToConstantHeading(
            Vector2d endPosition, double endTangent,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> new TrajectoryBuilder(currentPose, velConstraint, accelConstraint)
                .splineToConstantHeading(endPosition, endTangent)
                .build()
        );
    }

    /**
     * Adds a spline segment with linear heading interpolation
     * 
     * @param endPose the target pose
     * @param endTangent the ending tangent direction
     * @return this builder
     */
    public TrajectorySequenceBuilder splineToLinearHeading(Pose2d endPose, double endTangent) {
        return addPath(() -> new TrajectoryBuilder(currentPose, baseVelConstraint, baseAccelConstraint)
                .splineToLinearHeading(endPose, endTangent)
                .build()
        );
    }

    /**
     * Adds a spline segment with linear heading interpolation and custom constraints
     * 
     * @param endPose the target pose
     * @param endTangent the ending tangent direction
     * @param velConstraint the velocity constraint for this segment
     * @param accelConstraint the acceleration constraint for this segment
     * @return this builder
     */
    public TrajectorySequenceBuilder splineToLinearHeading(
            Pose2d endPose, double endTangent,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        return addPath(() -> new TrajectoryBuilder(currentPose, velConstraint, accelConstraint)
                .splineToLinearHeading(endPose, endTangent)
                .build()
        );
    }

    /**
     * Adds a turn segment that rotates in place
     * 
     * @param angle the angle to turn (in radians)
     * @return this builder
     */
    public TrajectorySequenceBuilder turn(double angle) {
        // Calculate turn duration based on the angular constraints
        double duration = Math.abs(angle) / baseTurnConstraintMaxAngVel;
        
        // Add the turn segment
        sequenceSegments.add(new TrajectorySequence.TurnSegment(
                currentPose, angle, duration
        ));
        
        // Update the current pose
        currentPose = new Pose2d(
                currentPose.getX(), currentPose.getY(),
                currentPose.getHeading() + angle
        );
        
        return this;
    }

    /**
     * Adds a wait segment that holds position for a time
     * 
     * @param seconds the time to wait
     * @return this builder
     */
    public TrajectorySequenceBuilder waitSeconds(double seconds) {
        sequenceSegments.add(new TrajectorySequence.WaitSegment(currentPose, seconds));
        return this;
    }

    /**
     * Helper method to add a trajectory path segment
     * 
     * @param trajectoryBuilder function that builds a trajectory
     * @return this builder
     */
    private TrajectorySequenceBuilder addPath(TrajectoryBuilderFunction trajectoryBuilder) {
        Trajectory trajectory = trajectoryBuilder.build();
        
        sequenceSegments.add(new TrajectorySequence.TrajectorySegment(trajectory));
        
        // Update the current pose
        currentPose = trajectory.end();
        
        return this;
    }

    /**
     * Builds the final trajectory sequence
     * 
     * @return the completed trajectory sequence
     */
    public TrajectorySequence build() {
        return new TrajectorySequence(sequenceSegments);
    }

    /**
     * Functional interface for building trajectories
     */
    private interface TrajectoryBuilderFunction {
        Trajectory build();
    }
}