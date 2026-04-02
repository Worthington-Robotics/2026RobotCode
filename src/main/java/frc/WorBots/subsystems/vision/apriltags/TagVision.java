
// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.vision.apriltags;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.vision.apriltags.TagVisionIO.DetectionMode;
import frc.WorBots.subsystems.vision.apriltags.TagVisionIO.TagVisionIOInputs;
import frc.WorBots.util.cache.Cache.TimeCache;
import frc.WorBots.util.debug.*;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.GeneralMath;
import frc.WorBots.util.math.GeomUtil;
import frc.WorBots.util.math.PoseEstimator.TimestampedVisionUpdate;
import java.util.*;
import java.util.function.*;

public class TagVision extends SubsystemBase {
  private final TagVisionIO[] io;
  private final TagVisionIOInputs[] inputs;

  /** Consumer that receives vision updates out of the subsystem */
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {
  };

  /** Supplier that gives the field-relative ChassisSpeeds of the robot */
  private Supplier<ChassisSpeeds> driveSpeedsSupplier = () -> new ChassisSpeeds();

  /** Supplier that provides the gyro yaw */
  private Supplier<Rotation2d> yawSupplier;

  /** Times of last detections for tags */
  private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  /** The amount of detections that have been made */
  private int detectionCount = 0;

  /** Whether the vision has seen a tag very recently */
  private boolean seesTag = false;

  /** Transform for the outward-facing camera on the left */
  private static final Transform3d LEFT_CAMERA_TRANSFORM = new Transform3d(
      new Translation3d(
          // x, y, and z
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(12.75),
          Units.inchesToMeters(13.5)),
      new Rotation3d(
          // TODO angle up is 30 degrees
          Units.degreesToRadians(0),
          Units.degreesToRadians(-30),
          Units.degreesToRadians(90)));

  // Transform for the outward-facing camera on the right
  private static final Transform3d RIGHT_CAMERA_TRANSFORM = new Transform3d(
      new Translation3d(
          Units.inchesToMeters(1),
          Units.inchesToMeters(-15.75),
          Units.inchesToMeters(13.5)),
      new Rotation3d(
          Units.degreesToRadians(0),
          Units.degreesToRadians(-30),
          Units.degreesToRadians(-93)));

  //Transform for the back-facing camera
  private static final Transform3d BACK_CAMERA_TRANSFORM = new Transform3d(
    new Translation3d(
      Units.inchesToMeters(-13.0),
      Units.inchesToMeters(-10.75),
      Units.inchesToMeters(7.0 + 3.0/8.0)),
    new Rotation3d(
      Units.degreesToRadians(0),
      Units.degreesToRadians(-30),
      Units.degreesToRadians(182)));

  /** The transforms for the cameras to robot center */
  private static final Transform3d[] CAMERA_TRANSFORMS = new Transform3d[] { LEFT_CAMERA_TRANSFORM,
      RIGHT_CAMERA_TRANSFORM, BACK_CAMERA_TRANSFORM };

  /** Latency between the camera and it being pushed to NT */
  private static final TunableDouble LATENCY = new TunableDouble("Vision", "Tuning", "AprilTag Latency", 0.0);

  /** Detection weights for each camera */
  private static final double[] CAMERA_WEIGHTS = new double[] { 1.0, 0.95, 1.0 };

  /**
   * How much influence XY data has on the robot pose. Smaller values increase
   * influence
   */
  // TODO: leave these where they are rn
  private static final double XY_STD_DEV_COEFFICIENT = 0.8;

  /**
   * How much influence theta data has on the robot pose. Smaller values increase
   * influence
   */
  private static final double THETA_STD_DEV_COEFFICIENT = 1.3;

  /**
   * The exponent for the distance scoring formula. Larger values make scores fall
   * off harder with
   * distance
   */
  private static final double DISTANCE_GAIN = 3.8;

  /** The margin inside the field border to accept vision poses from */
  private static final double FIELD_BORDER_MARGIN = 0.5;

  /**
   * The margin in the z-axis from 0m for a pose to be considered valid, in meters
   */
  private static final double Z_MARGIN = Units.inchesToMeters(26);

  /**
   * Weights for different tags on the field to be chosen. Only contains the
   * weights for one side
   * (half the tags)
   */
  // TODO: update/change these--meaning, maybe tune them?
  // The higher the value, the greater the trust in them.
  private static final double[] TAG_WEIGHTS = new double[] {
      0.75, // AprilTag 1
      1.0, // AprilTag 2
      1.0, // AprilTag 3
      1.0, // AprilTag 4
      1.0, // AprilTag 5
      0.75, // AprilTag 6
      0.75, // AprilTag 7
      1.0, // AprilTag 8
      1.0, // AprilTag 9
      1.0, // AprilTag 10
      1.0, // AprilTag 11
      0.75, // AprilTag 12
      1.0, // AprilTag 13
      1.0, // AprilTag 14
      1.0, // AprilTag 15
      1.0, // AprilTag 16
  };

  /** Factor for speed compensation */
  private static final double SPEED_COMPENSATION_MULTIPLIER = 0.0;

  /** The amount of time to log tag poses for */
  private static final double TARGET_LOG_TIME_SECS = 0.1;

  private final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
  private final BooleanPublisher isConnectedPublisher = visionTable.getBooleanTopic("Is Connected").publish();
  private final StructArrayPublisher<Pose2d> robotPosesPublisher = visionTable
      .getStructArrayTopic("RobotPoses", Pose2d.struct).publish();
  private final StructArrayPublisher<Pose3d> tagsPosesPublisher = visionTable
      .getStructArrayTopic("TagPoses", Pose3d.struct).publish();
  private final StructArrayPublisher<Pose3d> robotPoses3dPublisher = visionTable
      .getStructArrayTopic("RobotPoses3d", Pose3d.struct).publish();
  private final StructArrayPublisher<Pose3d> cameraPosesPublisher = visionTable
      .getStructArrayTopic("Camera Poses", Pose3d.struct).publish();
  private final DoublePublisher error0Publisher = visionTable.getDoubleTopic("Error 0").publish();
  private final DoublePublisher error1Publisher = visionTable.getDoubleTopic("Error 1").publish();
  private final DoublePublisher ambiguityPublisher = visionTable.getDoubleTopic("Ambiguity").publish();
  private final DoublePublisher finalScorePublisher = visionTable.getDoubleTopic("Final Score").publish();
  private final BooleanPublisher isPoseValidPublisher = visionTable.getBooleanTopic("Is Pose Valid").publish();
  private final StructPublisher<Pose3d> invalidPosePublisher = visionTable.getStructTopic("Invalid Pose", Pose3d.struct)
      .publish();
  private final StructArrayPublisher<Pose3d> cameraTransformsPublisher = visionTable.getStructArrayTopic("Camera Transforms", Pose3d.struct)
      .publish();
  private final IntegerPublisher detectionCountPublisher = visionTable.getIntegerTopic("Detection Count").publish();

  public TagVision(TagVisionIO... io) {
    this.io = io;
    inputs = new TagVisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new TagVisionIOInputs();
    }
    StatusPage.reportStatus(StatusPage.TAG_VISION_SUBSUBSYSTEM, true);
    Pose3d[] poses = new Pose3d[CAMERA_TRANSFORMS.length];
    for (int i = 0; i < CAMERA_TRANSFORMS.length; i++) {
      var pose = new Pose3d().plus(CAMERA_TRANSFORMS[i]);
      poses[i] = pose;
    }
    cameraTransformsPublisher.set(poses);
  }

  public void periodic() {
    // Update inputs
    boolean connected = true;
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      StatusPage.reportStatus(StatusPage.CAM_PREFIX + i, inputs[i].isConnected);
      if (!inputs[i].isConnected) {
        connected = false;
      }
    }
    StatusPage.reportStatus(StatusPage.TAG_VISION_SUBSUBSYSTEM, connected);

    seesTag = false;

    boolean isConnected = true;
    for (TagVisionIOInputs inputs : inputs) {
      if (!inputs.isConnected) {
        isConnected = false;
        break;
      }
    }
    isConnectedPublisher.set(isConnected);
    // TODO: put this back in after lights are done:
    // Lights.getInstance().setNoVisionIndicator(!isConnected);

    // Loop over instances
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<Pose3d> allCameraPoses = new ArrayList<>();
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();

    for (int camIndex = 0; camIndex < io.length; camIndex++) {
      final TagVisionIOInputs camInputs = inputs[camIndex];
      for (int frame = 0; frame < camInputs.timestamps.length; frame++) {
        final double timestamp = camInputs.timestamps[frame] - LATENCY.get();
        final double[] values = camInputs.frames[frame];
        if (values.length == 0 || values[0] == 0) {
          continue;
        }

        Pose3d cameraPose = null;
        Pose3d robotPose3d = null;
        double reprojectionError = 0.0;

        // Switch based on the number of detections
        switch ((int) values[0]) {
          case 1:
            // Multiple tags
            cameraPose = new Pose3d(
                values[2],
                values[3],
                values[4],
                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            break;
          case 2:
            // One tag that needs to be disambiguated
            final double error0 = values[1];
            final double error1 = values[9];

            error0Publisher.set(error0);
            error1Publisher.set(error1);

            // Select pose using either gyro or lower error based on the ambiguity ratio
            final double ambiguity = Math.min(error0, error1) / Math.max(error0, error1);
            ambiguityPublisher.set(ambiguity);

            final Pose3d pose0 = new Pose3d(
                values[2],
                values[3],
                values[4],
                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            final Pose3d pose1 = new Pose3d(
                values[10],
                values[11],
                values[12],
                new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
            ;

            // From 6328
            if (yawSupplier == null || ambiguity < 0.4) {
              // Choose pose with lower reprojection error
              if (error0 < error1) {
                cameraPose = pose0;
                reprojectionError = error0;
              } else {
                cameraPose = pose1;
                reprojectionError = error1;
              }
            } else {
              // Choose pose that is closer to the gyro angle
              final double realRotation = AllianceFlipUtil.apply(yawSupplier.get()).getRadians();
              final double diff0 = Math.abs(pose0.getRotation().getZ() - realRotation);
              final double diff1 = Math.abs(pose1.getRotation().getZ() - realRotation);
              if (diff0 < diff1) {
                cameraPose = pose0;
                reprojectionError = error0;
              } else {
                cameraPose = pose1;
                reprojectionError = error1;
              }
            }

            break;
        }

        // Set robot pose
        if (cameraPose != null) {
          var camTransform = CAMERA_TRANSFORMS[camIndex];
          // robotPose3d = new Pose3d(cameraPose.getTranslation().minus(camTransform.getTranslation()),
          //     cameraPose.getRotation().minus(camTransform.getRotation()));
          robotPose3d = cameraPose.transformBy(camTransform.inverse());
        }

        // Exit if no data
        if (cameraPose == null || robotPose3d == null) {
          continue;
        }

        // Exit if the pose is invalid
        if (!isPoseValid(robotPose3d)) {
          continue;
        }

        seesTag = true;

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        List<Integer> tagIds = new ArrayList<>();
        // TODO: Change these numbers according to AprilTags
        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
          final int tagId = (int) values[i];
          tagIds.add(tagId);

          lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());

          final Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose(tagId);
          if (tagPose.isPresent()) {
            tagPoses.add(tagPose.get());
          }
        }

        // Get 2D robot pose
        Pose2d robotPose;
        if (!tagPoses.isEmpty()) {
          robotPose = groundRobotPose(robotPose3d, GeomUtil.averagePose3ds(tagPoses));
        } else {
          robotPose = robotPose3d.toPose2d();
        }

        // Speed-compensate the robot pose
        final ChassisSpeeds driveSpeeds = driveSpeedsSupplier.get();
        robotPose = GeomUtil.applyChassisSpeeds(robotPose, driveSpeeds, SPEED_COMPENSATION_MULTIPLIER);

        // Calculate average score from all tag detections
        double averageScore = tagIds.size() > 0 ? getTagScore(tagIds.get(0)) : 1.0;
        averageScore *= CAMERA_WEIGHTS[camIndex];
        finalScorePublisher.set(averageScore);

        // Add to vision updates with the calculated standard deviation of the update
        if (averageScore > 0.0) {
          double xStdDev = 0.001;
          double yStdDev = 0.001;
          double thetaStdDev = 0.00001;
          if (frame < camInputs.covariances.length) {
            Pose3d covariance = camInputs.covariances[frame];
            xStdDev = XY_STD_DEV_COEFFICIENT * covariance.getX() / averageScore;
            yStdDev = XY_STD_DEV_COEFFICIENT * covariance.getY() / averageScore;
            thetaStdDev = THETA_STD_DEV_COEFFICIENT * covariance.getRotation().getZ() / averageScore;
          }
          SmartDashboard.putNumber("X Dev", xStdDev);
          SmartDashboard.putNumber("Y Dev", yStdDev);
          SmartDashboard.putNumber("Rot Dev", thetaStdDev);
          visionUpdates.add(
              new TimestampedVisionUpdate(
                  timestamp, robotPose, VecBuilder.fill(xStdDev, yStdDev, thetaStdDev)));
        }

        allRobotPoses.add(robotPose);
        allRobotPoses3d.add(robotPose3d);
        allCameraPoses.add(cameraPose);

        detectionCount++;
        detectionCountPublisher.set(detectionCount);
      }

      // Collect all tag poses, log them, and update whether we have seen a tag
      List<Pose3d> allTagPoses = new ArrayList<>();
      for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
        if (TimeCache.getInstance().get() - detectionEntry.getValue() < TARGET_LOG_TIME_SECS) {
          allTagPoses.add(FieldConstants.aprilTags.getTagPose(detectionEntry.getKey()).get());
          seesTag = true;
        }
      }
      seesTag = true;

      // Log poses
      setRobotPoses(allRobotPoses);
      setRobotPoses3d(allRobotPoses3d);
      setCameraPoses(allCameraPoses);
      setTagPoses(allTagPoses);

      // Send vision data to consumer
      visionConsumer.accept(visionUpdates);
    }
  }

  /**
   * This function accepts the interfaces in and out of the vision system, such as
   * giving out vision
   * updates, and recieving poses.
   */
  public void setDataInterfaces(
      Consumer<List<TimestampedVisionUpdate>> visionConsumer,
      Supplier<Rotation2d> yawSupplier,
      Supplier<ChassisSpeeds> driveSpeedsSupplier) {
    this.visionConsumer = visionConsumer;
    this.yawSupplier = yawSupplier;
    this.driveSpeedsSupplier = driveSpeedsSupplier;
  }

  /** Checks if the vision can currently see a tag */
  public boolean canSeeTag() {
    return seesTag;
  }

  /** Sets the vision into multi-tag mode */
  public void setMultiTagMode() {
    for (TagVisionIO io : io) {
      io.setDetectionConfig(DetectionMode.MultiTag, -1);
    }
  }

  /** Sets the vision into specific tag mode */
  public void setSpecificTagMode(int tag) {
    for (TagVisionIO io : io) {
      io.setDetectionConfig(DetectionMode.SpecificTag, tag);
    }
  }

  /**
   * Gets whether a 3D pose from vision should be considered valid
   *
   * @param pose The pose from vision
   * @return Whether the pose should be used
   */
  private boolean isPoseValid(Pose3d pose) {
    if (pose.getX() < -FIELD_BORDER_MARGIN
        || pose.getX() > FieldConstants.fieldLength + FIELD_BORDER_MARGIN
        || pose.getY() < -FIELD_BORDER_MARGIN
        || pose.getY() > FieldConstants.fieldWidth + FIELD_BORDER_MARGIN
        || pose.getZ() < -Z_MARGIN
        || pose.getZ() > Z_MARGIN) {
      invalidPosePublisher.set(pose);
      isPoseValidPublisher.set(false);
      return false;
    }
    isPoseValidPublisher.set(true);

    return true;
  }

  /**
   * Scores a single tag detection
   *
   * @param error    The reprojection error of the detection
   * @param distance The distance to the AprilTag
   * @param camID    The ID of the camera making the detection
   * @param tagID    The ID of the AprilTag (starting from 1). If -1 or 0, the tag
   *                 score will be
   *                 ignored
   * @return The score. This value is unitless and relative to other scores only
   */
  private static double scoreDetection(double error, double distance, int camID, int tagID) {
    double score = 1.0;

    // A higher error decreases the score
    score -= error * 0.25;
    if (score < 0.0) {
      return 0.0;
    }

    // A higher distance decreases the score
    score /= GeneralMath.curve(distance, DISTANCE_GAIN);

    // Weigh based on the camera
    final double camWeight = CAMERA_WEIGHTS[camID];
    score *= camWeight;

    // Weigh based on the tag
    if (tagID > 0 && tagID <= TAG_WEIGHTS.length / 2) {
      final int index = (tagID - 1) % TAG_WEIGHTS.length;
      final double tagWeight = TAG_WEIGHTS[index];
      score *= tagWeight;
    }

    return score;
  }

  /**
   * Grounds a 3d robot pose from tag detection into a 2d one with more accuracy
   *
   * @param robot The robot pose
   * @param tag   The tag pose
   * @return The 2D robot pose
   */
  private static Pose2d groundRobotPose(Pose3d robot, Pose3d tag) {
    final double dx = robot
        .getTranslation()
        .toTranslation2d()
        .getDistance(tag.getTranslation().toTranslation2d());
    final double dz = robot.getZ();
    final double hypotenuse = Math.hypot(dx, dz);
    final double difference = hypotenuse - dx;
    SmartDashboard.putNumber("Vision Grounding Difference", difference);
    final Translation2d translation = GeomUtil.moveAwayFrom(
        robot.getTranslation().toTranslation2d(),
        tag.getTranslation().toTranslation2d(),
        difference * -1.0);
    return new Pose2d(translation, robot.getRotation().toRotation2d());
  }

  private double getTagScore(int id) {
    if (id > 0 && id <= TAG_WEIGHTS.length / 2) {
      final int index = (id - 1) % TAG_WEIGHTS.length;
      return TAG_WEIGHTS[index];
    } else {
      return 1.0;
    }
  }

  /** Set the robot poses publisher */
  private void setRobotPoses(List<Pose2d> poses) {
    robotPosesPublisher.set(poses.toArray(new Pose2d[0]));
  }

  /** Set the 3D robot poses publisher */
  private void setRobotPoses3d(List<Pose3d> poses) {
    robotPoses3dPublisher.set(poses.toArray(new Pose3d[0]));
  }

  /** Set the 3D robot poses publisher */
  private void setCameraPoses(List<Pose3d> poses) {
    cameraPosesPublisher.set(poses.toArray(new Pose3d[0]));
  }

  /** Set the tag poses publisher */
  private void setTagPoses(List<Pose3d> poses) {
    tagsPosesPublisher.set(poses.toArray(new Pose3d[0]));
  }

}
