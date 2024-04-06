package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

public class AllianceUtil {
  private static AllianceColor alliance = AllianceColor.UNKNOWN;
  private static boolean configured = false;
  private static Supplier<Pose2d> robotPose;
  private static double fieldLength;
  private static double fieldHeight;

  /** Whether the field is mirrored (true) or rotated (false). */
  private static boolean mirroredField;

  public enum AllianceColor {
    RED,
    BLUE,
    UNKNOWN;
  }

  public enum FieldDesignType {
    ROTATED,
    MIRRORED;
  }

  class NotConfiguredException extends NullPointerException {
    public NotConfiguredException(String s) {
      super(s);
    }
  }

  public static void configure(
      Supplier<Pose2d> robotPose, double fieldLength, double fieldHeight, boolean mirroredField) {
    AllianceUtil.robotPose = robotPose;
    AllianceUtil.fieldLength = fieldLength;
    AllianceUtil.fieldHeight = fieldHeight;
    AllianceUtil.mirroredField = mirroredField;
    configured = true;
  }

  public static void setAlliance() {
    if (DriverStation.getAlliance().isEmpty()) {
      alliance = AllianceColor.UNKNOWN;
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      alliance = AllianceColor.BLUE;
    } else if (DriverStation.getAlliance().get() == Alliance.Red) {
      alliance = AllianceColor.RED;
    } else {
      alliance = AllianceColor.UNKNOWN;
    }
  }

  /**
   * Maps a blue pose to the red side based on configured field information.
   *
   * @param bluePose The blue pose to map to red.
   * @return The red pose mapped from the blue pose.
   */
  public static Pose2d mapBluePoseToRed(Pose2d bluePose) {
    if (!configured)
      throw new NullPointerException(
          "The field and robot pose have not been configured. Please call AllianceUtil.configure() before this method.");
    if (mirroredField) {
      return new Pose2d(
          fieldLength - bluePose.getX(),
          bluePose.getY(),
          new Rotation2d(-(bluePose.getRotation().getRadians() - (Math.PI / 2)) + (Math.PI / 2)));
    } else {
      return new Pose2d(
          fieldLength - bluePose.getX(),
          fieldHeight - bluePose.getY(),
          new Rotation2d(bluePose.getRotation().getRadians() + Math.PI));
    }
  }

  public static AllianceColor getAlliance() {
    return alliance;
  }

  public static Pose2d getPoseForAlliance(Pose2d bluePose) {
    Pose2d redPose = mapBluePoseToRed(bluePose);
    if (alliance.equals(AllianceColor.BLUE)) {
      return bluePose;
    } else if (alliance.equals(AllianceColor.RED)) {
      return redPose;
    } else {
      if (robotPose.get().getTranslation().getDistance(bluePose.getTranslation())
          < robotPose.get().getTranslation().getDistance(redPose.getTranslation())) {
        return bluePose;
      } else {
        return redPose;
      }
    }
  }
}
