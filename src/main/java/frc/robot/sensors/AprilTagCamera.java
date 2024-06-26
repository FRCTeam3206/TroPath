package frc.robot.sensors;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class AprilTagCamera {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  SwerveDrivePoseEstimator swervePoseEstimator;
  private double lastEstTimestamp = 0;

  public AprilTagCamera(
      String cameraName, Transform3d distToCamera, SwerveDrivePoseEstimator swervePoseEstimator) {
    camera = new PhotonCamera(cameraName);
    photonEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kAprilTagFieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            distToCamera);
    photonEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    this.swervePoseEstimator = swervePoseEstimator;
  }

  public void addVisionMeasurementToEstimator() {
    var visionEst = getEstimatedGlobalPose();
    if (visionEst.isPresent()) {
      Pose2d estPose2d = visionEst.get().estimatedPose.toPose2d();
      Matrix<N3, N1> estStdDevs = getEstimationStandardDeviations(estPose2d);
      swervePoseEstimator.addVisionMeasurement(
          estPose2d, visionEst.get().timestampSeconds, estStdDevs);
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  public Matrix<N3, N1> getEstimationStandardDeviations(Pose2d estPose) {
    var estStandardDeviations = VisionConstants.kSingleTagStandardDeviations;
    var targets = camera.getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estPose.getTranslation());
    }
    avgDist /= numTags;

    if (numTags == 0) {
      return estStandardDeviations;
    }

    // if (numTags > 1) {
    //   estStandardDeviations = VisionConstants.kMultiTagStandardDeviations;
    // } else if (numTags == 1 && avgDist > 4) {
    //   estStandardDeviations = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE,
    // Double.MAX_VALUE);
    // } else {
    estStandardDeviations = estStandardDeviations.times(1 + (avgDist * avgDist / 30));
    // }

    return estStandardDeviations;
  }
}
