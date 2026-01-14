// package frc.robot.subsystems;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import frc.robot.LimelightHelpers;

// public class Vision {
//     private final String limelightName;
//     private final SwerveSubSystem swerve;
    
//     // Tuning constants (adjust these as needed)
//     private static final double DISTANCE_THRESHOLD = 0.5 ; // meters; stop when within this distance
//     private static final double TRANSLATION_SCALE = -0.24;  // scale factor for x and y movement
//     private static final double ROTATION_SCALE = 0.3;       // scale factor for rotational correction
//     private static final double ANGLE_THRESHOLD = 0.05;     // radians; ignore small rotational errors

//     public Vision(SwerveSubSystem swerve, String limelightName) {
//         this.swerve = swerve;
//         this.limelightName = limelightName;
//     }

//     // Default constructor
//     public Vision(SwerveSubSystem swerve) {
//         this(swerve, "limelight");
//     }

//     /**
//      * Gets the latest AprilTag pose.
//      * @return The robot’s estimated position relative to the AprilTag.
//      */
//     public Pose3d getAprilTagPose() {
//         return LimelightHelpers.getBotPose3d_wpiBlue(limelightName);
//     }

//     public double getX(){
//         Pose3d tagPose = getAprilTagPose();
//         return tagPose.getX() - 16.46;
//     }

//     public double getY()
//     {
//         Pose3d tagPose = getAprilTagPose();
//         return tagPose.getY() - 0.89;
//     }

//     public double getZ()
//     {
//         Pose3d tagPose = getAprilTagPose();
//         return tagPose.getZ();
//     }

//     public double getDistance()
//     {
//         return this.getY() * this.getY() + this.getX() + this.getX();
//     }

//     /**
//      * Calculates the movement needed to approach and align to the detected AprilTag.
//      * Stops when the robot is close enough.
//      * @return ChassisSpeeds to be applied to the swerve drive.
//      */
//     public ChassisSpeeds getAprilTagMovement() {
//         Pose3d tagPose = getAprilTagPose();
        
//         if (tagPose != null) {
//             double x = tagPose.getX() - 16.46; // Forward/backward distance
//             double y = tagPose.getY() - 0.89; // Left/right distance
//             System.out.println("x pos of tagPose"+tagPose.getX());
//             System.out.println("y pos of tagPose"+tagPose.getY());
//             System.out.println("z pos of tagPose"+tagPose.getY());
//             double distance = Math.sqrt(x * x + y * y);
            
//             // Stop movement if the robot is within the threshold distance
//             if (distance < DISTANCE_THRESHOLD) 
//             {
                
//                 return new ChassisSpeeds(0, 0, 0);
//             }
            
//             // Calculate translation speeds with scaling
//             double forwardSpeed = x * TRANSLATION_SCALE;
//             double strafeSpeed = y * TRANSLATION_SCALE;
            
//             // Calculate rotational speed based on the detected angle error.
//             // Here, we use the z-component of the Rotation3d as an approximation of the yaw error.
//             double angleError = tagPose.getRotation().getZ() - 1.288;
//             double rotationalSpeed = 0.0;
//             if (Math.abs(angleError) > ANGLE_THRESHOLD) {
//                 rotationalSpeed = angleError * ROTATION_SCALE;
//             }
            
//             return new ChassisSpeeds(forwardSpeed*0.3, strafeSpeed*0.3, rotationalSpeed*0.3);
//         }
        
//         return new ChassisSpeeds(0, 0, 0); // No target detected, so stop
//     }

//     /**
//      * Moves the swerve drive toward and aligns to the detected AprilTag.
//      */
//     public void moveTowardAprilTag() {
//         swerve.driveFieldOriented(getAprilTagMovement());
//     }

//     /**
//      * Stops the swerve drive.
//      */
//     public void stop() {
//         swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
//     }

//     /**
//      * Returns a command to align to an AprilTag.
//      * This command continually moves the robot until it is close enough, then stops.
//      */
//     public Command getAlignToAprilTagCommand(SwerveSubSystem drivebase) {
//         return new RunCommand(() -> moveTowardAprilTag(), drivebase)
//             .until(() -> {
//                 Pose3d tagPose = getAprilTagPose();
//                 if (tagPose != null) {
//                     double x = tagPose.getX();
//                     double y = tagPose.getY();
//                     double distance = Math.sqrt(x * x + y * y);
//                     return distance < DISTANCE_THRESHOLD;
//                 }
//                 return false;
//             })
//             .finallyDo(interrupted -> stop());
//     }
// }
