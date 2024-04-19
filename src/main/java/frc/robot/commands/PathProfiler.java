package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class PathProfiler {
  private double maxVelocity, maxAcceleration;

  public PathProfiler(double maxVelocity, double maxAcceleration) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    System.out.println("Max velocity " + maxVelocity);
  }

  Comparator<ProfiledPathPoint> pathPointVelocityComparator =
      new Comparator<ProfiledPathPoint>() {

        @Override
        public int compare(ProfiledPathPoint o1, ProfiledPathPoint o2) {
          return o1.velocity > o2.velocity ? 1 : -1;
        }
      };

  public double getNextRobotSpeed(
      double inVelocity, Pose2d currentRobotPose, ArrayList<Pose2d> path) {
    ArrayList<ProfiledPathPoint> profiledPath = new ArrayList<>();
    profiledPath.add(new ProfiledPathPoint(0).setLocation(path.get(0)).setVelocity(maxVelocity));
    int i = 1;
    for (; i < path.size(); i++) {
      profiledPath.add(
          new ProfiledPathPoint(i).setLocation(path.get(i)).setLast(profiledPath.get(i - 1)));
      profiledPath.get(i - 1).setNext(profiledPath.get(i));
      if (i == 1) continue;
      double stopDist =
          poseDist(profiledPath.get(i - 1), profiledPath.get(i)) / angle(profiledPath.get(i - 1));
      double maxAllowedVelocity = Math.sqrt(stopDist * 2 * maxAcceleration);
      if (Double.isInfinite(maxAllowedVelocity) || Double.isNaN(maxAllowedVelocity)) {
        maxAllowedVelocity = maxVelocity;
      }
      profiledPath.get(i - 1).setVelocity(Math.min(maxAllowedVelocity, maxVelocity));
    }
    int maxIndex = i - 1;
    profiledPath.get(maxIndex).setVelocity(0);
    Collections.sort(profiledPath, pathPointVelocityComparator);
    ProfiledPathPoint currentPoint = profiledPath.get(0);
    while (currentPoint.getIndex() != 0) {
      if (currentPoint.getIndex() != maxIndex
          && profiledPath.contains(currentPoint.getNext())) { // We can check above
        ProfiledPathPoint nextPoint = currentPoint.getNext();
        nextPoint.setVelocity(
            Math.min(
                nextPoint.getVelocity(),
                Math.sqrt(
                    currentPoint.getVelocity() * currentPoint.getVelocity()
                        + 2 * maxAcceleration * poseDist(nextPoint, currentPoint))));
        int index = profiledPath.indexOf(nextPoint);
        profiledPath.remove(index);
        profiledPath.add(binaryVelocitySearch(profiledPath.subList(0, index), nextPoint), nextPoint);
      }
      if (profiledPath.contains(currentPoint.getLast())) {
        ProfiledPathPoint lastPoint = currentPoint.getLast();
        lastPoint.setVelocity(
            Math.min(
                lastPoint.getVelocity(),
                Math.sqrt(
                    currentPoint.getVelocity() * currentPoint.getVelocity()
                        + 2 * maxAcceleration * poseDist(lastPoint, currentPoint))));
        if (lastPoint.getIndex() == 0) { // Optimized stopping
          currentPoint = lastPoint;
          break;
        }
        int index = profiledPath.indexOf(lastPoint);
        profiledPath.remove(index);
        profiledPath.add(binaryVelocitySearch(profiledPath.subList(0, index), lastPoint), lastPoint);
      }
      profiledPath.remove(currentPoint);
      currentPoint = profiledPath.get(0);
    }
    return Math.min(currentPoint.getVelocity(), inVelocity + maxAcceleration * .02);
  }

  // Code for binary search courtesy of chat-gpt
  public static int binaryVelocitySearch(List<ProfiledPathPoint> list, ProfiledPathPoint item) {
    int low = 0;
    int high = list.size() - 1;

    while (low <= high) {
      int mid = (low + high) / 2;
      ProfiledPathPoint midItem = list.get(mid);

      if (midItem.getVelocity() < item.getVelocity()) {
        low = mid + 1; // Search in the right half
      } else {
        high = mid - 1; // Search in the left half
      }
    }

    return low;
  }

  private double angle(ProfiledPathPoint p1, ProfiledPathPoint p2, ProfiledPathPoint p3) {
    double d1 = poseDist(p1, p2);
    double d2 = poseDist(p2, p3);
    double d3 = poseDist(p3, p1);
    return Math.PI - Math.acos((d1 * d1 + d2 * d2 - d3 * d3) / (2 * d1 * d2));
  }

  private double angle(ProfiledPathPoint pt) {
    return angle(pt.getLast(), pt, pt.getNext());
  }

  private double poseDist(ProfiledPathPoint p1, ProfiledPathPoint p2) {
    return p1.getLocation().getTranslation().getDistance(p2.getLocation().getTranslation());
  }

  private class ProfiledPathPoint {
    private double velocity;
    private ProfiledPathPoint last, next;
    private Pose2d location;
    private int index = -1;

    public ProfiledPathPoint(int i) {
      index = i;
    }

    public double getVelocity() {
      return velocity;
    }

    public ProfiledPathPoint setVelocity(double velocity) {
      this.velocity = velocity;
      return this;
    }

    public ProfiledPathPoint getLast() {
      return last;
    }

    public ProfiledPathPoint setLast(ProfiledPathPoint last) {
      this.last = last;
      return this;
    }

    public ProfiledPathPoint getNext() {
      return next;
    }

    public ProfiledPathPoint setNext(ProfiledPathPoint next) {
      this.next = next;
      return this;
    }

    public Pose2d getLocation() {
      return location;
    }

    public ProfiledPathPoint setLocation(Pose2d location) {
      this.location = location;
      return this;
    }

    public int getIndex() {
      return index;
    }

    public String toString() {
      return "ProfiledPathPoint [velocity="
          + velocity
          + ", location="
          + location
          + ", index="
          + index
          + "]";
    }
  }
}
