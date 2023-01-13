package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;
import java.util.OptionalDouble;

import static frc.robot.Constants.Vision.*;

public class PhotonVisionModule extends SubsystemBase {
    private final PhotonCamera camera;

    public PhotonVisionModule() {
        camera = new PhotonCamera("borusancam");
    }

    /**
     * Check whether the camera detected a target.
     *
     * @return whether we have a target.
     */
    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * Gets the distance of the robot from the target.
     *
     * @return the distance of the vision module from the target. [m]
     */
    public OptionalDouble getDistance() {
        var results = camera.getLatestResult();
        if (results.hasTargets()) {
            return OptionalDouble
                    .of(PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT, TARGET_HEIGHT_FROM_GROUND,
                            Math.toRadians(CAMERA_PITCH), Math.toRadians(results.getBestTarget().getPitch())));
        }
        return OptionalDouble.empty();
    }

    /**
     * Estimates the camera translation relative to the target.
     *
     * @return the translation relative to the target.
     */
    public Optional<Translation2d> estimateCameraTranslationToTarget() {
        PhotonPipelineResult results;
        results = camera.getLatestResult();
        if (results.hasTargets()) {
            double distance = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT, TARGET_HEIGHT_FROM_GROUND,
                    Math.toRadians(CAMERA_PITCH), Math.toRadians(results.getBestTarget().getPitch()));
            return Optional.of(PhotonUtils.estimateCameraToTargetTranslation(distance,
                    Rotation2d.fromDegrees(-results.getBestTarget().getYaw())));
        }
        return Optional.empty();
    }

    /**
     * Estimates the pose of the robot.
     *
     * @return the estimated pose and the time of detection.
     */
    public Optional<VisionEstimationData> estimatePose() {
        PhotonPipelineResult res;
        res = camera.getLatestResult();
        if (res.hasTargets()) {
            double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis() / 1000.0;
            Transform2d camToTargetTrans = res.getBestTarget().getCameraToTarget();
            Pose2d camPose = HUB_POSE.transformBy(camToTargetTrans.inverse());
            return Optional.of(new VisionEstimationData(camPose, imageCaptureTime));
        }
        return Optional.empty();
    }

    @Override
    public void periodic() {
        Optional<Translation2d> toTarget = estimateCameraTranslationToTarget();
        if (toTarget.isPresent()) {
            SmartDashboard.putNumber("to target x", toTarget.get().getX());
            SmartDashboard.putNumber("to target y", toTarget.get().getY());
        }
        SmartDashboard.putBoolean("hasTarget", hasTargets());
    }
}