// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;

public class CameraSubsystem extends SubsystemBase {
  PhotonCamera m_camera1;
  PhotonCamera m_camera2;
  PhotonPoseEstimator m_photonPoseEstimatorCam1;
  PhotonPoseEstimator m_photonPoseEstimatorCam2;
  PhotonPipelineResult m_resultCam1;
  PhotonPipelineResult m_resultCam2;
  Optional<PhotonTrackedTarget> m_lowestAmbiguityTarget;

  AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  //Constructor
  public CameraSubsystem(String Camera1Name, String Camera2Name) {
    m_camera1 = new PhotonCamera(Camera1Name);
    m_camera2 = new PhotonCamera(Camera2Name);

    m_photonPoseEstimatorCam1 = 
    new PhotonPoseEstimator(
      m_aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      m_camera1,
      PhotonVisionConstants.kRobotToCamera1Transform
    );
    
    m_photonPoseEstimatorCam2 =
    new PhotonPoseEstimator(
      m_aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      m_camera2,
      PhotonVisionConstants.kRobotToCamera2Transform
    );

    m_photonPoseEstimatorCam1.setTagModel(TargetModel.kAprilTag36h11);
    m_photonPoseEstimatorCam2.setTagModel(TargetModel.kAprilTag36h11);
  }

  public PoseStrategy getEstimatorStrategy() {
    return m_photonPoseEstimatorCam1.getPrimaryStrategy();
  }

  public boolean hasTargetsCam1() {
    return m_resultCam1.hasTargets();
  }

  public boolean hasTargetsCam2() {
    return m_resultCam2.hasTargets();
  }

  public List<PhotonTrackedTarget> getAllTargets() {
    List<PhotonTrackedTarget> combinedList = m_resultCam1.getTargets();
    combinedList.addAll(m_resultCam2.getTargets());
    return combinedList;
  }

  public void takeSnapshotCam1() {
    m_camera1.takeInputSnapshot();
  }

  public void takeSnapshotCam2() {
    m_camera2.takeInputSnapshot();
  }

  public ArrayList<Optional<EstimatedRobotPose>> getFieldRelativePoseEstimators() {
    ArrayList<Optional<EstimatedRobotPose>> estimatorList = new ArrayList<Optional<EstimatedRobotPose>>();
    estimatorList.add(m_photonPoseEstimatorCam1.update(m_resultCam1));
    estimatorList.add(m_photonPoseEstimatorCam2.update(m_resultCam2));
    return estimatorList;
  }

  public Pose3d getFieldRelativePose(PhotonTrackedTarget target) {
    return PhotonUtils.estimateFieldToRobotAprilTag(
        target.getBestCameraToTarget(),
        m_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
        PhotonVisionConstants.kRobotToCamera1Transform
      );
  }

  private Optional<PhotonTrackedTarget> getLowestAmbiguityTargetImpl(List<PhotonTrackedTarget> targetList) {
    Optional<PhotonTrackedTarget> bestTarget = Optional.empty();
    for (int i=0; i<targetList.size(); i++) {
      PhotonTrackedTarget currentTarget = targetList.get(i);
      if (currentTarget.getPoseAmbiguity() == -1 || currentTarget.getPoseAmbiguity() >= 0.2) continue;
      if (bestTarget.isEmpty()) {
        bestTarget = Optional.of(currentTarget);
        continue;
      }
      if (currentTarget.getPoseAmbiguity() < bestTarget.get().getPoseAmbiguity()) {
        bestTarget = Optional.of(currentTarget);
      }
    }
    return bestTarget;
  }

  public Optional<PhotonTrackedTarget> getLowestAmbiguityTarget() {
    return m_lowestAmbiguityTarget;
  }

  @Override
  public void periodic() {
    m_resultCam1 = m_camera1.getLatestResult();
    m_resultCam2 = m_camera2.getLatestResult();
    m_lowestAmbiguityTarget = getLowestAmbiguityTargetImpl(getAllTargets());
  }
}
