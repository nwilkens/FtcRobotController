package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.Collections;
import java.util.List;

/**
 * A sequence of trajectories that can be executed as a single unit.
 * This allows for more complex path following, including waiting periods,
 * turns in place, and other actions between trajectory segments.
 */
public class TrajectorySequence {
    // List of segments that make up this sequence
    private final List<SequenceSegment> sequenceSegments;

    /**
     * Constructs a trajectory sequence from a list of segments
     * @param sequenceSegments list of trajectory segments
     */
    public TrajectorySequence(List<SequenceSegment> sequenceSegments) {
        // Check if the sequence is empty
        if (sequenceSegments.size() == 0) {
            throw new IllegalArgumentException("Trajectory sequence cannot be empty");
        }
        
        this.sequenceSegments = Collections.unmodifiableList(sequenceSegments);
    }

    /**
     * Gets the start pose of the trajectory sequence
     * @return the starting pose
     */
    public Pose2d start() {
        return sequenceSegments.get(0).getStartPose();
    }

    /**
     * Gets the end pose of the trajectory sequence
     * @return the ending pose
     */
    public Pose2d end() {
        return sequenceSegments.get(sequenceSegments.size() - 1).getEndPose();
    }

    /**
     * Gets all segments in this sequence
     * @return list of segments
     */
    public List<SequenceSegment> getSequenceSegments() {
        return sequenceSegments;
    }

    /**
     * Gets the total duration of this trajectory sequence
     * @return total duration in seconds
     */
    public double duration() {
        double total = 0.0;
        
        for (SequenceSegment segment : sequenceSegments) {
            total += segment.getDuration();
        }
        
        return total;
    }

    /**
     * Base segment interface that all segment types must implement
     */
    public interface SequenceSegment {
        /**
         * Gets the duration of this segment
         * @return duration in seconds
         */
        double getDuration();

        /**
         * Gets the starting pose of this segment
         * @return starting pose
         */
        Pose2d getStartPose();

        /**
         * Gets the ending pose of this segment
         * @return ending pose
         */
        Pose2d getEndPose();
    }

    /**
     * A segment that consists of a standard Roadrunner trajectory
     */
    public static class TrajectorySegment implements SequenceSegment {
        private final Trajectory trajectory;

        public TrajectorySegment(Trajectory trajectory) {
            this.trajectory = trajectory;
        }

        @Override
        public double getDuration() {
            return trajectory.duration();
        }

        @Override
        public Pose2d getStartPose() {
            return trajectory.start();
        }

        @Override
        public Pose2d getEndPose() {
            return trajectory.end();
        }

        public Trajectory getTrajectory() {
            return trajectory;
        }
    }

    /**
     * A segment that represents a turn in place
     */
    public static class TurnSegment implements SequenceSegment {
        private final Pose2d pose;
        private final double totalRotation;
        private final double duration;

        public TurnSegment(Pose2d pose, double totalRotation, double duration) {
            this.pose = pose;
            this.totalRotation = totalRotation;
            this.duration = duration;
        }

        @Override
        public double getDuration() {
            return duration;
        }

        @Override
        public Pose2d getStartPose() {
            return pose;
        }

        @Override
        public Pose2d getEndPose() {
            return new Pose2d(
                    pose.getX(), pose.getY(),
                    pose.getHeading() + totalRotation
            );
        }

        public double getTotalRotation() {
            return totalRotation;
        }
    }

    /**
     * A segment that waits for a specified duration
     */
    public static class WaitSegment implements SequenceSegment {
        private final Pose2d pose;
        private final double duration;

        public WaitSegment(Pose2d pose, double duration) {
            this.pose = pose;
            this.duration = duration;
        }

        @Override
        public double getDuration() {
            return duration;
        }

        @Override
        public Pose2d getStartPose() {
            return pose;
        }

        @Override
        public Pose2d getEndPose() {
            return pose;
        }
    }
}