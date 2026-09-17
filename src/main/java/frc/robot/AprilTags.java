package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class AprilTags {
    private Transform3d frontLeftCamPos; 
    private Transform3d backRightCamPos; 

    public AprilTags() {
        frontLeftCamPos = new Transform3d(
            Units.inchesToMeters(12.75),
            Units.inchesToMeters(7.5),
            Units.inchesToMeters(7.875),
            new Rotation3d(Units.degreesToRadians(52), Units.degreesToRadians(0), Units.degreesToRadians(78))
        );
    }

}
