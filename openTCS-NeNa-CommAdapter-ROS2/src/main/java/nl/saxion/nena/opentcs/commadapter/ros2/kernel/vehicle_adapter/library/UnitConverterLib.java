package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library;

import geometry_msgs.msg.Quaternion;
import org.opentcs.data.model.Triple;

import javax.annotation.Nonnull;

/**
 * Library class for converting distance and rotation units.
 *
 * @author Niels Tiben
 */
public abstract class UnitConverterLib {
    public static double convertMillimetersToMeters(long millimeters) {
        double millimetersDouble = (double) millimeters;

        return millimetersDouble / 1000;
    }

    public static double[] convertTripleToCoordinatesInMeter(@Nonnull Triple triple) {
        double x = convertMillimetersToMeters(triple.getX());
        double y = convertMillimetersToMeters(triple.getY());
        double z = convertMillimetersToMeters(triple.getZ());

        return new double[]{x, y, z};
    }

    public static Triple convertCoordinatesInMeterToTriple(double x, double y, double z) {
        long xInMillimeter = convertMetersToMillimeters(x);
        long yInMillimeter = convertMetersToMillimeters(y);
        long zInMillimeter = convertMetersToMillimeters(z);

        return new Triple(xInMillimeter, yInMillimeter, zInMillimeter);
    }

    private static long convertMetersToMillimeters(double meters) {
        double millimeters = meters * 1000;

        return Math.round(millimeters);
    }

    public static double quaternionToAngleDegree(@Nonnull Quaternion quaternion) {
        // Convert quaternion to angle radians
        double angleRad = 2.0 * Math.acos(quaternion.getZ());

        // Inverse angle radian
        angleRad = angleRad * -1;

        // Convert angle radians to angle degrees
        double angleDegree = Math.toDegrees(angleRad);

        // Add 180° to comply with OpenTCS way of storing orientations (n=90°; e=0°; s=270°; w=180°)
        angleDegree = angleDegree + 180;

        // Normalize degree between 0° and 360°
        angleDegree = angleDegree % 360;
        if (angleDegree < 0) {
            angleDegree += 360;
        }

        return angleDegree;
    }

    public static geometry_msgs.msg.Quaternion VehicleOrientationToQuaternion(@Nonnull double vehicleorientation) {
        /* ROS TF provides function to convert RPY to Quaternion, but TF is not ported to the RCL..
        http://docs.ros.org/melodic/api/tf/html/c++/namespacetf.html#a09ddb5ff50f5032761feaa6a760f7344

        therefore we go to the implementation and do a similar implementation, keeping in mind that only the Yaw is relevant as the other angles are zero.
        http://docs.ros.org/melodic/api/tf/html/c++/Quaternion_8h_source.html#l00096
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
         */

        double yaw = Math.toRadians(vehicleorientation);
        geometry_msgs.msg.Quaternion q = new geometry_msgs.msg.Quaternion();
        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = 1.0;
        double sp = 0.0;
        double cr = 1.0;
        double sr = 0.0;

        q.setW(cr * cp * cy + sr * sp * sy);
        q.setX(sr * cp * cy - cr * sp * sy);
        q.setY( cr * sp * cy + sr * cp * sy);
        q.setZ(cr * cp * sy - sr * sp * cy);

        return q;
    }
}
