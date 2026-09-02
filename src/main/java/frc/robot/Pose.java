package frc.robot;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class Pose {

SwerveDrivePoseEstimator poseEstimator;

    public Pose(SwerveDriveKinematics kinematics, 
                SwerveModulePosition[] modulePositions,
                Rotation2d robotYaw) {
        

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            robotYaw,
            modulePositions,
            new Pose2d(0, 0, new Rotation2d(0)));     
    }

    public void updatePoseEstimator(Rotation2d currentRotation, 
                                    SwerveModulePosition[] currentPosition) {

        poseEstimator.update(currentRotation, currentPosition);
    }
}
