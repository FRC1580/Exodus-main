package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.LimelightHelpers;

public class Vision {
    private final String limelightName;
    private final SwerveSubSystem swerve;

    // Tuning constants
    // Use a positive distance threshold (e.g., 0.25 meters)
    private static final double DISTANCE_THRESHOLD_METERS = 0.005;
    // Translation speeds are scaled
    private static final double TRANSLATION_SCALE = 1; // tune as needed
    // Yaw/rotation correction
    private static final double ROTATION_SCALE = 1.5; // tune as needed
    // Angle error threshold (radians) to start/stop rotating
    private static final double ANGLE_THRESHOLD_RADIANS = 0.03;

    public Vision(SwerveSubSystem swerve, String limelightName) {
        this.swerve = swerve;
        this.limelightName = limelightName;
        LimelightHelpers.setPipelineIndex(limelightName, 0); // AprilTag pipeline by default
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    // Default constructor uses "limelight" as the limelight name
    public Vision(SwerveSubSystem swerve) {
        this(swerve, "limelight");
    }

    /**
     * Gets the latest AprilTag pose in the camera frame.
     * @return Pose3d of the tag in camera space, or null if not available
     */
    public Pose3d getAprilTagPose() {
        // Be defensive: may return null if no target
        return LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
    }
    /*
     * <summary>
     * gets the horizontal angle to the apriltag in relation to the camera in radians
     * </summary>
     * @return double angle in radians
     */
    public double getAngleRadians() {
        Pose3d tagPose = getAprilTagPose();
        return (Math.atan2(tagPose.getX(),tagPose.getZ()));
    }

    /*
     * <summary>
     * gets the position of the apriltag in relation to the camera in the horizontal axis
     * </summary>
     * @return double X position in meters//TO DO: check correct measurement
     */
    public double getX() {
        Pose3d tagPose = getAprilTagPose();
        return tagPose != null ? tagPose.getX() : 0.0;
    }

    /*
     * <summary>
     * gets the position of the apriltag in relation to the camera in the vertical axis
     * </summary>
     * @return double Y position in meters//TO DO: check correct measurement
     */

    public double getY() {
        Pose3d tagPose = getAprilTagPose();
        return tagPose != null ? tagPose.getY() : 0.0;
    }

    /*
     * <summary>
     * gets the position of the apriltag in relation to the camera in the forward/backward direction
     * </summary>
     * @return double Z position in meters//TO DO: check correct measurement
     */

    public double getZ() {
        Pose3d tagPose = getAprilTagPose();
        return tagPose != null ? tagPose.getZ() : 0.0;
    }

    // Distance to the tag in the plane (ignorin g height)
    public double getDistanceMeters() {
        Pose3d tagPose = getAprilTagPose();
        if (tagPose == null) return -1;
        double x = tagPose.getX();
        double y = tagPose.getY();
        return Math.hypot(x, y); // sqrt(x^2 + z^2)

    }

    /**
     * Calculates the movement needed to approach and align to the detected AprilTag.
     * Stops when the robot is within DISTANCE_THRESHOLD_METERS.
     * @return ChassisSpeeds to be applied to the swerve drive.
     */
    public ChassisSpeeds getAprilTagMovement() {
        Pose3d tagPose = getAprilTagPose();
        if (tagPose == null) {
            // No target detected; stop
            return new ChassisSpeeds(0, 0, 0);
        }

        double x = tagPose.getX();
        double y = tagPose.getY();

        // Compute planar distance to tag
        double distance = Math.hypot(x, y);

        // Stop if within threshold
        if (distance <= DISTANCE_THRESHOLD_METERS) {
            return new ChassisSpeeds(0, 0, 0);
        }

        // Translate toward tag (scale applied)
        // Depending on your field orientation, you may need to invert x/y.
        double forwardSpeed = x * TRANSLATION_SCALE; // negative if positive x means tag is ahead
        double strafeSpeed  = y * TRANSLATION_SCALE; // adjust sign as needed for your frame

        // Rotation: try to align yaw to face the tag (use Z of Rotation3d as yaw)
        double yawError = getAngleRadians(); // radians
        double rotationalSpeed = 0.0;
        rotationalSpeed = -yawError * ROTATION_SCALE;
        

        // Optional small cap on speeds to avoid overshoot
        double maxAbs = 1.0;
        
        // If you want to bias linear velocities a bit (e.g., reduce when rotation is high),
        // you can apply additional logic here.

        return new ChassisSpeeds(0, 0, rotationalSpeed);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /**
     * Moves the swerve drive toward and aligns to the detected AprilTag.
     */
    public void moveTowardAprilTag() {
        swerve.driveFieldOriented(getAprilTagMovement());
    }

    /**
     * Stops the swerve drive.
     */
    public void stop() {
        swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }
    /**
     * Change this to add whatever changes or updates you want to happen periodically. (20ms)
     */
    public void update() {
        Pose3d tagPose = getAprilTagPose();
        if (tagPose != null && Math.abs(getAngleRadians()) > ANGLE_THRESHOLD_RADIANS) {
            moveTowardAprilTag();
        }
    }

}