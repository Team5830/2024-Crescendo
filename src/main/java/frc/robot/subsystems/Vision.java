// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
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
    private PhotonPipelineResult result;
    private List<PhotonTrackedTarget> targets;
    public List<PhotonTrackedTarget> matched = new ArrayList<PhotonTrackedTarget>(1);
    private int currentTag;
    public Vision() {
        try {
            camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating vision: " + ex.getMessage(), true);
        }
    }

    //Use to update camera results for specified AprilTag, other methods return info from matched result
    public void getAprilTagVisionResult(int tagNumber ) {
        matched.clear();
        targets.clear();
        currentTag = -1;
        result = camera.getLatestResult();
        //Get requested Apriltag from field layout
        targets = result.getTargets();
        for (PhotonTrackedTarget t : targets){
            if(t.getFiducialId()==tagNumber){
                // If requested tag is found put result in matched
                matched.add(t);
                currentTag=tagNumber;
            };
        }
    }

    public Transform3d getAprilTagTransform(){
        if (!matched.isEmpty()){
            return matched.get(0).getBestCameraToTarget();
        }
        return new Transform3d();
    }

    public Pose3d getAprilTagPose(int targetTag){
        return(aprilTagFieldLayout.getTagPose(targetTag).get());
    }

    public double getAprilTagRange() {
        Pose3d tagPose;   
        try{
            //Get requested Apriltag from field layout to find height
            tagPose = aprilTagFieldLayout.getTagPose(currentTag).get();
        } catch (NoSuchElementException ex) {
            DriverStation.reportError("Error getting Tag Pose: " + ex.getMessage(), true);
            return 10000.0; // Return huge number?  since out of range
        }
        if (!matched.isEmpty()){
            PhotonTrackedTarget t = matched.get(0);
            return PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.vision.CAMERA_HEIGHT_METERS,
                    tagPose.getZ(),
                    Constants.vision.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(t.getPitch())
                    );
        }
        return 10000.0; // Return huge number?  since out of range
    }

    public double getAprilTagYaw() {
        if (!matched.isEmpty()){
            PhotonTrackedTarget t = matched.get(0);
            return t.getYaw();
        }
        return 1000;
    }
    
    @Override
    public void periodic() {
    }
}
