// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.VisionResult;

public class Vision extends SubsystemBase {
    PhotonCamera camera;
    PIDController forwardController;
    PIDController turnController;

    public Vision() {
        try {
            camera = new PhotonCamera("photonvision");

            forwardController = new PIDController(Constants.vision.linearP, Constants.vision.linearI,
                    Constants.vision.linearD);
            turnController = new PIDController(Constants.vision.angularP, Constants.vision.angularI,
                    Constants.vision.angularD);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating vision: " + ex.getMessage(), true);
        }
    }

    public VisionResult calculateTargetMovement() {
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.vision.CAMERA_HEIGHT_METERS,
                    Constants.vision.TARGET_HEIGHT_METERS,
                    Constants.vision.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));

            return new VisionResult(-forwardController.calculate(range, Constants.vision.goalRangeMeters),
                    -turnController.calculate(result.getBestTarget().getYaw(), 0));
        }

        return new VisionResult();
    }

    @Override
    public void periodic() {
    }
}
