package org.firstinspires.ftc.teamcode.jules.core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Interprets dynamic path commands from JULES App and builds Pedro PathChains
 * at runtime.
 * This eliminates the need for code rebuilds when iterating on autonomous
 * paths.
 *
 * Usage flow:
 * 1. path_start  -> reset()
 * 2. path_add    -> addSegment(...) (called N times, one per segment)
 * 3. path_follow -> execute(follower)
 *
 * Geometry:
 *   - controlPoints empty -> BezierLine(start, end)
 *   - controlPoints size N -> BezierCurve(start, cp1, ..., cpN, end)  (Pedro variadic)
 *
 * Heading interpolation (per segment):
 *   - "linear"     -> setLinearHeadingInterpolation(startDeg, endDeg)
 *   - "tangential" -> setTangentHeadingInterpolation()  (+ optional setReversed)
 *   - "constant"   -> setConstantHeadingInterpolation(headingDeg)
 */
public class JulesPathInterpreter {
    private static final String TAG = "JulesPathInterpreter";

    /** Represents one path segment before it is compiled. */
    public static class Segment {
        public final String type; // linear, tangential, constant
        public final double x;
        public final double y;
        public final double startHeading; // Linear only
        public final double endHeading; // Linear only
        public final double heading; // Tangential / Constant
        public final boolean reverse; // Tangential only
        public final List<Pose> controlPoints; // [] = line, else Bezier of order N+1

        public Segment(String type, double x, double y,
                double startHeading, double endHeading,
                double heading, boolean reverse,
                List<Pose> controlPoints) {
            this.type = type;
            this.x = x;
            this.y = y;
            this.startHeading = startHeading;
            this.endHeading = endHeading;
            this.heading = heading;
            this.reverse = reverse;
            this.controlPoints = (controlPoints == null)
                    ? Collections.<Pose>emptyList()
                    : controlPoints;
        }
    }

    private final List<Segment> segments = new ArrayList<>();
    private boolean pathActive = false;

    /**
     * Optional path origin from the app ({@code path_start} or {@code path_add} start marker).
     * Heading is degrees (same as segment heading fields); converted to radians for Pedro {@link Pose}.
     */
    private boolean hasExplicitStartPose = false;
    private double explicitStartX;
    private double explicitStartY;
    private double explicitStartHeadingDeg;

    /** Reset builder for a new path chain. */
    public void reset() {
        segments.clear();
        pathActive = true;
        hasExplicitStartPose = false;
        RobotLog.i(TAG, "Path interpreter reset - ready for segments");
    }

    /**
     * Sets the pose used as the first point of the chain when {@link #execute(Follower)} runs,
     * instead of {@link Follower#getPose()}. Also syncs the follower via {@code setPose}.
     *
     * @param headingDeg field-heading style degrees (consistent with path segment JSON)
     */
    public void setStartPose(double x, double y, double headingDeg) {
        explicitStartX = x;
        explicitStartY = y;
        explicitStartHeadingDeg = headingDeg;
        hasExplicitStartPose = true;
        RobotLog.i(TAG, "Explicit start pose: (" + x + ", " + y + ") headingDeg=" + headingDeg);
    }

    /** Add a straight (BezierLine) segment — backward-compatible overload. */
    public void addSegment(String type, double x, double y,
            double startHeading, double endHeading,
            double heading, boolean reverse) {
        addSegment(type, x, y, startHeading, endHeading, heading, reverse, null);
    }

    /** Add a segment to the current path (line or bezier curve). */
    public void addSegment(String type, double x, double y,
            double startHeading, double endHeading,
            double heading, boolean reverse,
            List<Pose> controlPoints) {
        if (!pathActive) {
            reset();
        }
        segments.add(new Segment(type, x, y, startHeading, endHeading,
                heading, reverse, controlPoints));
        int cps = (controlPoints == null) ? 0 : controlPoints.size();
        RobotLog.i(TAG, "Added segment: " + type + " (" + x + ", " + y + ") cps=" + cps);
    }

    /** Build and execute the accumulated path chain on the given follower. */
    public boolean execute(Follower follower) {
        if (follower == null) {
            RobotLog.w(TAG, "Cannot execute: Follower is null");
            return false;
        }
        if (segments.isEmpty()) {
            RobotLog.w(TAG, "Cannot execute: No segments defined");
            return false;
        }

        try {
            Pose currentPose;
            if (hasExplicitStartPose) {
                currentPose = new Pose(explicitStartX, explicitStartY,
                        Math.toRadians(explicitStartHeadingDeg));
                follower.setPose(currentPose);
                hasExplicitStartPose = false;
            } else {
                currentPose = follower.getPose();
                if (currentPose == null) {
                    currentPose = new Pose(0, 0, 0);
                }
            }

            PathBuilder builder = follower.pathBuilder();
            Pose lastPoint = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());

            for (Segment seg : segments) {
                Pose endPoint = new Pose(seg.x, seg.y, currentPose.getHeading());

                // Geometry: line vs bezier curve
                if (seg.controlPoints == null || seg.controlPoints.isEmpty()) {
                    builder.addPath(new BezierLine(lastPoint, endPoint));
                } else {
                    // BezierCurve takes variadic Point... — start, cps..., end
                    Pose[] pts = new Pose[seg.controlPoints.size() + 2];
                    pts[0] = lastPoint;
                    for (int i = 0; i < seg.controlPoints.size(); i++) {
                        pts[i + 1] = seg.controlPoints.get(i);
                    }
                    pts[pts.length - 1] = endPoint;
                    builder.addPath(new BezierCurve(pts));
                }

                // Heading interpolation
                switch (seg.type.toLowerCase()) {
                    case "linear":
                        builder.setLinearHeadingInterpolation(
                                Math.toRadians(seg.startHeading),
                                Math.toRadians(seg.endHeading));
                        break;
                    case "tangential":
                        builder.setTangentHeadingInterpolation();
                        if (seg.reverse) {
                            builder.setReversed();
                        }
                        break;
                    case "constant":
                        builder.setConstantHeadingInterpolation(Math.toRadians(seg.heading));
                        break;
                    default:
                        RobotLog.w(TAG, "Unknown segment type: " + seg.type);
                        builder.setTangentHeadingInterpolation();
                        break;
                }

                lastPoint = endPoint;
            }

            PathChain chain = builder.build();
            follower.followPath(chain, true);
            RobotLog.i(TAG, "Path chain built & following (" + segments.size() + " segments)");

            segments.clear();
            pathActive = false;
            return true;

        } catch (Exception e) {
            RobotLog.e(TAG, "Path execution failed: " + e.getMessage());
            segments.clear();
            pathActive = false;
            return false;
        }
    }

    public int getSegmentCount() {
        return segments.size();
    }

    public boolean isActive() {
        return pathActive;
    }
}
