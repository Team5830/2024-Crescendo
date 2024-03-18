// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    private PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    public Optional<PhotonTrackedTarget> matched;
    private int tagLastFound = -1;
    private int tagToFind = -1; 

    public Vision() {
        try {
            camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating vision: " + ex.getMessage(), true);
        }
    }

    // Update tag to find 
    public void getAprilTagVisionResult(int tagNumber) {
        tagToFind=tagNumber;
        updateSearchResult();
    }

    // Use to update camera results for specified AprilTag, other methods return
    // info from matched result
    public void updateSearchResult(){
        var result = camera.getLatestResult();
        // Get requested Apriltag from field layout
        var targets = result.getTargets();
        for (PhotonTrackedTarget t : targets) {
            if (t.getFiducialId() == tagToFind) {
                // If requested tag is found put result in matched
                matched = Optional.of(t);
                tagLastFound = tagToFind;
                break;
            }
        }
    }


    public Transform3d getAprilTagTransform() {
        if (matched.isPresent()) {
            return matched.get().getBestCameraToTarget();
        }
        return new Transform3d();
    }

    public Pose3d getAprilTagPose(int targetTag) {
        return (aprilTagFieldLayout.getTagPose(targetTag).get());
    }

    public double getAprilTagRange() {
        Pose3d tagPose;
        try {
            // Get requested Apriltag from field layout to find height
            tagPose = aprilTagFieldLayout.getTagPose(tagLastFound).get();
        } catch (NoSuchElementException ex) {
            DriverStation.reportError("Error getting Tag Pose: " + ex.getMessage(), true);
            return Double.NaN; // Return NaN, since out of range
        }
        if (matched.isPresent()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.vision.CAMERA_HEIGHT_METERS,
                    tagPose.getZ(),
                    Constants.vision.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(matched.get().getPitch()));
        }
        return Double.NaN; // Return NaN, since out of range
    }

    public double getAprilTagYaw() {
        if (matched.isPresent()) {
            return matched.get().getYaw();
        }
        return Double.NaN; // Return NaN, since out of range
    }

    @Override
    public void periodic() {
        updateSearchResult();
        SmartDashboard.putNumber("Last Tag", tagLastFound);
        SmartDashboard.putNumber("Range to Tag", getAprilTagRange());
        SmartDashboard.putNumber("Yaw to Tag", getAprilTagYaw());
    }
}
