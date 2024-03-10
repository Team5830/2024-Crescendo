// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.VisionResult;

public class Vision extends SubsystemBase {
    PhotonCamera camera;
    PIDController forwardController;
    PIDController turnController;
    AprilTagFieldLayout aprilTagFieldLayout;
    
    public Vision() {
        
        try {
            camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating vision: " + ex.getMessage(), true);
        }
    }

    public List<PhotonTrackedTarget> getAprilTagVisionResult(int tagNumber ) {
        var result = camera.getLatestResult();
        //Get requested Apriltag from field layout
        List<PhotonTrackedTarget> targets = result.getTargets();
        List<PhotonTrackedTarget> matched = new ArrayList<PhotonTrackedTarget>(1);
        for (PhotonTrackedTarget t : targets){
            if(t.getFiducialId()==tagNumber){
                // If requested tag is found return its relative position as result
                matched.add(t);
                return matched;
            };
        }
        return matched;
    }

    public Transform3d getAprilTagTransform(int tagNumber){
        List<PhotonTrackedTarget> targets = getAprilTagVisionResult(tagNumber);
        if (!targets.isEmpty()){
            return targets.get(0).getBestCameraToTarget();
        }
        return new Transform3d();
    }

    public double getAprilTagRange(int tagNumber ) {
        List<PhotonTrackedTarget> targets = getAprilTagVisionResult(tagNumber);
        Pose3d tagPose;   
        try{
            //Get requested Apriltag from field layout
            tagPose = aprilTagFieldLayout.getTagPose(tagNumber).get();
        } catch (NoSuchElementException ex) {
            DriverStation.reportError("Error getting Tag Pose: " + ex.getMessage(), true);
            return 10000.0; // Return huge number?  since out of range
        }
        if (!targets.isEmpty()){
            PhotonTrackedTarget t = targets.get(0);
            return PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.vision.CAMERA_HEIGHT_METERS,
                    tagPose.getZ(),
                    Constants.vision.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(t.getPitch())
                    );
        }
        return 10000.0;
    }

    public double getAprilTagYaw(int tagNumber ) {
        List<PhotonTrackedTarget> targets = getAprilTagVisionResult(tagNumber);
        if (!targets.isEmpty()){
            PhotonTrackedTarget t = targets.get(0);
            return t.getYaw();
        }
        return 1000;
    }
    
    /*
    
    public Translation2d getTranslationToTag(int tagNumber){
        List<PhotonTrackedTarget> targets = getAprilTagVisionResult(tagNumber);
        Pose3d tagPose;   
        try{
            //Get requested Apriltag from field layout
            tagPose = aprilTagFieldLayout.getTagPose(tagNumber).get();
        } catch (NoSuchElementException ex) {
            DriverStation.reportError("Error getting Tag Pose: " + ex.getMessage(), true);
            return Translation2d() ; // Return huge number?  since out of range
        }
            //Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(range, Rotation2d.fromDegrees(-t.getYaw()));
                //robotpose?  is this actual current pose on field, or is it fixed in robot frame?
                //Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, tagPose);
            //return new VisionResult(forwardController.calculate(range, Constants.vision.goalRangeMeters),-turnController.calculate(result.getBestTarget().getYaw(), 0));
    } 

    public Transform3d getAprilTagTransform(int tagNumber ) {
        var result = camera.getLatestResult();
        Pose3d tagPose;
        try{
            //Get requested Apriltag from field layout
            tagPose = aprilTagFieldLayout.getTagPose(tagNumber).get();
        } catch (NoSuchElementException ex) {
            DriverStation.reportError("Error getting Tag Pose: " + ex.getMessage(), true);
            return new Transform3d();
        }
        // Get all visible tags
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget t : targets){
            if(t.getFiducialId()==tagNumber){
                // If requested tag is found return its relative position as result
                return t.getBestCameraToTarget();
            };
        }
        return new Transform3d();
    }
    

    public double getAprilTagRange(int tagNumber ) {
        var result = camera.getLatestResult();
        Pose3d tagPose;
        try{
            //Get requested Apriltag from field layout
            tagPose = aprilTagFieldLayout.getTagPose(tagNumber).get();
        } catch (NoSuchElementException ex) {
            DriverStation.reportError("Error getting Tag Pose: " + ex.getMessage(), true);
            return 10000.0; // Return huge number?  since out of range
        }
        // Get all visible tags
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget t : targets){
            if(t.getFiducialId()==tagNumber){
                double range = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.vision.CAMERA_HEIGHT_METERS,
                    tagPose.getZ(),
                    Constants.vision.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(t.getPitch())
                    );
                SmartDashboard.putNumber("vision.range", range);
                return range;
            }
        }
        return 10000.0; // Return huge number?  since out of range
    }
    */

    //public VisionResult gVisionResult(int tagNumber){

            //Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(range, Rotation2d.fromDegrees(-t.getYaw()));
                //robotpose?  is this actual current pose on field, or is it fixed in robot frame?
                //Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, tagPose);
            //return new VisionResult(forwardController.calculate(range, Constants.vision.goalRangeMeters),-turnController.calculate(result.getBestTarget().getYaw(), 0));
    //}
    @Override
    public void periodic() {
    }
}
