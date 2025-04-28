package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

/**
 * Core class for executing trajectory sequences.
 * Manages all the state tracking and segment execution for complex paths.
 */
public class TrajectorySequenceRunner {
    // Execution state tracking
    private final TrajectoryFollower follower; // For trajectory segments
    private final PIDFController turnController; // For turn segments
    
    private final NanoClock clock = NanoClock.system();
    private TrajectorySequence currentTrajectorySequence;
    private double currentSegmentStartTime;
    private int currentSegmentIndex;
    private int lastSegmentIndex;
    
    // Pose estimation and error tracking
    private Pose2d lastPoseError = new Pose2d();
    private Pose2d poseEstimate = new Pose2d();
    private Pose2d poseVelocity = new Pose2d();

    /**
     * Constructor for the trajectory sequence runner
     * @param follower trajectory follower for trajectory segments
     * @param initialPose initial pose of the robot
     */
    public TrajectorySequenceRunner(TrajectoryFollower follower, Pose2d initialPose) {
        this.follower = follower;
        
        // Create a turn controller for turn segments
        // Use PID coefficients from DriveConstants for heading control
        turnController = new PIDFController(new PIDCoefficients(
            DriveConstants.HEADING_PID_COEFF_P,
            DriveConstants.HEADING_PID_COEFF_I,
            DriveConstants.HEADING_PID_COEFF_D
        ));
        turnController.setInputBounds(0, 2 * Math.PI);
        
        setCurrentPose(initialPose);
    }

    /**
     * Sets the current pose estimate
     * @param pose the pose to set
     */
    public void setCurrentPose(Pose2d pose) {
        poseEstimate = pose;
    }
    
    // We're going to use getLastPoseError() and poseEstimate
    // without creating a duplicate getPoseEstimate() method

    /**
     * Follows a new trajectory sequence
     * @param trajectorySequence the sequence to follow
     */
    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        currentTrajectorySequence = trajectorySequence;
        currentSegmentStartTime = clock.seconds();
        currentSegmentIndex = 0;
        lastSegmentIndex = -1;
    }

    /**
     * Follows a trajectory sequence asynchronously
     * @param trajectorySequence the sequence to follow
     */
    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        followTrajectorySequence(trajectorySequence);
    }

    /**
     * Cancels the current trajectory sequence execution
     */
    public void cancelTrajectorySequence() {
        currentTrajectorySequence = null;
    }

    /**
     * Updates the trajectory follower with the current pose
     * @param poseEstimate current estimated pose
     * @param poseVelocity current estimated velocity (optional)
     * @return the drive signal to execute, or null if not following a trajectory
     */
    public DriveSignal update(Pose2d poseEstimate, Pose2d poseVelocity) {
        this.poseEstimate = poseEstimate;
        this.poseVelocity = poseVelocity;

        // If we're not following a trajectory, return null
        if (currentTrajectorySequence == null) {
            return new DriveSignal();
        }

        // Get the list of segments from the current sequence
        List<TrajectorySequence.SequenceSegment> segments = currentTrajectorySequence.getSequenceSegments();
        
        // If we've run out of segments, we're done
        if (currentSegmentIndex >= segments.size()) {
            currentTrajectorySequence = null;
            return new DriveSignal();
        }

        // Get the current segment
        double now = clock.seconds();
        double segmentElapsedTime = now - currentSegmentStartTime;
        TrajectorySequence.SequenceSegment currentSegment = segments.get(currentSegmentIndex);

        // Start a new segment if needed
        if (currentSegmentIndex != lastSegmentIndex) {
            // For trajectory segments, reset the follower
            if (currentSegment instanceof TrajectorySequence.TrajectorySegment) {
                Trajectory trajectory = ((TrajectorySequence.TrajectorySegment) currentSegment).getTrajectory();
                follower.followTrajectory(trajectory);
            }
            
            // For turn segments, reset the turn controller
            else if (currentSegment instanceof TrajectorySequence.TurnSegment) {
                turnController.reset();
                double targetOmega = ((TrajectorySequence.TurnSegment) currentSegment).getTotalRotation() / currentSegment.getDuration();
                turnController.setTargetVelocity(targetOmega);
                turnController.setTargetPosition(currentSegment.getEndPose().getHeading());
            }

            lastSegmentIndex = currentSegmentIndex;
        }

        // If the segment is complete, move to the next one
        if (segmentElapsedTime >= currentSegment.getDuration()) {
            currentSegmentStartTime = now;
            currentSegmentIndex++;
            lastSegmentIndex = -1; // Force a reset for the next segment
            
            // If we have more segments, process the next one immediately
            if (currentSegmentIndex < segments.size()) {
                return update(poseEstimate, poseVelocity);
            } else {
                currentTrajectorySequence = null;
                return new DriveSignal();
            }
        }

        // Handle the specific segment type
        if (currentSegment instanceof TrajectorySequence.TrajectorySegment) {
            // Use the trajectory follower
            Trajectory trajectory = ((TrajectorySequence.TrajectorySegment) currentSegment).getTrajectory();
            
            // Rather than trying to sample directly, we'll let the follower handle the trajectory
            // and just track the pose error against the current pose
            Pose2d targetPose = follower.getLastError().plus(poseEstimate);
            lastPoseError = targetPose.minus(poseEstimate);
            
            return follower.update(poseEstimate, poseVelocity);
        } 
        else if (currentSegment instanceof TrajectorySequence.TurnSegment) {
            // Use the turn controller
            double targetOmega = ((TrajectorySequence.TurnSegment) currentSegment).getTotalRotation() / currentSegment.getDuration();
            targetOmega = segmentElapsedTime >= currentSegment.getDuration() ? 0 : targetOmega;
            
            double targetHeading = currentSegment.getStartPose().getHeading() + 
                    ((TrajectorySequence.TurnSegment) currentSegment).getTotalRotation() * 
                    (segmentElapsedTime / currentSegment.getDuration());
                    
            turnController.setTargetPosition(targetHeading);
            
            double correction = turnController.update(poseEstimate.getHeading(), targetOmega);
            double targetOmegaX = 0;
            double targetOmegaY = 0;
            
            return new DriveSignal(
                    new Pose2d(targetOmegaX, targetOmegaY, correction)
            );
        } 
        else if (currentSegment instanceof TrajectorySequence.WaitSegment) {
            // Just wait in place
            return new DriveSignal();
        }
        
        throw new RuntimeException("Unknown segment type: " + currentSegment.getClass().getSimpleName());
    }

    /**
     * Checks if the runner is currently following a trajectory sequence
     * @return true if following a sequence
     */
    public boolean isBusy() {
        return currentTrajectorySequence != null;
    }

    /**
     * Gets the current pose estimate
     * @return the current pose estimate
     */
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    /**
     * Gets the current pose velocity estimate
     * @return the current pose velocity estimate
     */
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    /**
     * Gets the error between the target pose and the actual pose
     * @return the pose error
     */
    public Pose2d getLastPoseError() {
        return lastPoseError;
    }
}