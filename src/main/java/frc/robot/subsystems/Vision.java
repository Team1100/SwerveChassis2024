// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private PhotonCamera m_camera;
  private PhotonPoseEstimator m_photonEstimator;
  private double m_lastEstTime;

  /** Creates a new Vision. */
  public Vision() {
    m_camera = new PhotonCamera("AprilTagCamera");

    m_photonEstimator = new PhotonPoseEstimator(
                          VisionConstants.kTagLayout,
                          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                          m_camera,
                          VisionConstants.kRobotToCam);
    
    m_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    
  }

  @Override
  public void periodic() {
    Drive robotDrive = Drive.getInstance();

    var newest = getEstimatedGlobalPose();
    newest.ifPresent(
      est -> {
        Pose2d estPose = est.estimatedPose.toPose2d();
        var stdDevs = getEstimationStdDevs(estPose);

        robotDrive.addVisionMeasurement(estPose, m_lastEstTime, stdDevs);
      }
    );

  }

  public PhotonPipelineResult getLatestResult() {
    return m_camera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = m_photonEstimator.update();
    double latestTimestamp = m_camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - m_lastEstTime) > 1e-5;

    if (newResult) m_lastEstTime = latestTimestamp;
    return visionEst;
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = targets.size();
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = m_photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
}
