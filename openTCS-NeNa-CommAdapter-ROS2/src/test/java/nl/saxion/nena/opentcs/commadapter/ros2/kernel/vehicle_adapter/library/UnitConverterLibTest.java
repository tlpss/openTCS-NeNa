package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library;

import geometry_msgs.msg.Quaternion;
import org.junit.Test;
import org.opentcs.data.model.Triple;

import java.util.Arrays;

/**
 * @author Niels Tiben
 */
public class UnitConverterLibTest {
    @Test
    public void testConvertMillimetersToMeters() {
        long distanceInMillimeters = 1555;
        double distanceInMeters = UnitConverterLib.convertMillimetersToMeters(distanceInMillimeters);

        assert distanceInMeters == 1.555;
    }

    @Test
    public void testConvertTripleToCoordinatesInMeter() {
        Triple tripleInMillimeter = new Triple(1010, 2020, 3030);
        double[] coordinatesInMeter = UnitConverterLib.convertTripleToCoordinatesInMeter(tripleInMillimeter);

        assert Arrays.equals(coordinatesInMeter, new double[]{1.01, 2.02, 3.03});
    }

    @Test
    public void testConvertCoordinatesInMeterToTriple() {
        double[] coordinatesInMeter = new double[]{1.01, 2.02, 3.03};
        Triple tripleInMillimeter = UnitConverterLib.convertCoordinatesInMeterToTriple(
                coordinatesInMeter[0],
                coordinatesInMeter[1],
                coordinatesInMeter[2]
        );

        assert tripleInMillimeter.equals(new Triple(1010, 2020, 3030));
    }

    @Test
    public void testQuaternionToAngleDegreeNorth(){
        Quaternion quaternionTowardsNorth = new Quaternion()
                .setX(0)
                .setY(0)
                .setZ(0.71) // approximate north
                .setW(0.71); // approximate north

        double angleTowardsNorth = UnitConverterLib.quaternionToAngleDegree(quaternionTowardsNorth);
        assert angleTowardsNorth > 89 && angleTowardsNorth < 91; // approximate north
    }

    @Test
    public void testQuaternionToAngleDegreeWest(){
        Quaternion quaternionTowardsNorth = new Quaternion()
                .setX(0)
                .setY(0)
                .setZ(1) // exact west
                .setW(0); // exact west

        double angleTowardsWest = UnitConverterLib.quaternionToAngleDegree(quaternionTowardsNorth);
        assert angleTowardsWest == 180; // exact west
    }

    @Test
    public void testOrientationToQuaternion(){
        //case 1 - just take random orientation out of running OpenTCS instance
        double degrees = 14;
        Quaternion q = new    Quaternion().setX(0).setY(0).setW(0.99).setZ(0.12);

        double orientation = Math.toRadians(degrees);
        geometry_msgs.msg.Quaternion q_converted  = UnitConverterLib.VehicleOrientationToQuaternion(degrees);
        assert Math.abs(q.getX()-q_converted.getX()) < 0.01;
        assert Math.abs(q.getY()-q_converted.getY()) < 0.01;
        assert Math.abs(q.getZ()-q_converted.getZ()) < 0.01;
        assert Math.abs(q.getW()-q_converted.getW()) < 0.01;

    }
}
