package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Radians;

import com.studica.frc.Navx;

public class Drive {
    private Wheel frontLeft;
    private Wheel frontRight;
    private Wheel backLeft;
    private Wheel backRight;
    private Navx  gyro;

    //Constants
    private final int FRONT_LEFT_DRIVE_CAN_ID  = 14;
    private final int FRONT_RIGHT_DRIVE_CAN_ID = 16;
    private final int BACK_LEFT_DRIVE_CAN_ID   = 12;
    private final int BACK_RIGHT_DRIVE_CAN_ID  = 10;

    private final int FRONT_LEFT_ROTATE_CAN_ID  = 15;
    private final int FRONT_RIGHT_ROTATE_CAN_ID = 17;
    private final int BACK_LEFT_ROTATE_CAN_ID   = 13;
    private final int BACK_RIGHT_ROTATE_CAN_ID  = 11;

    private final int GYRO_CAN_ID = 3;

    private final double WHEEL_DIST_FROM_CENTER_INCH = 12.8125; //12 5/8 inches distance between wheels
    private final double WHEEL_DIST_FROM_CENTER_METERS = Units.inchesToMeters(WHEEL_DIST_FROM_CENTER_INCH);

    private final Translation2d frontLeftWheelLocation  = new Translation2d( WHEEL_DIST_FROM_CENTER_METERS, 
                                                                             WHEEL_DIST_FROM_CENTER_METERS);
    private final Translation2d frontRightWheelLocation = new Translation2d( WHEEL_DIST_FROM_CENTER_METERS, 
                                                                            -WHEEL_DIST_FROM_CENTER_METERS);                                                                          
    private final Translation2d backLeftWheelLocation   = new Translation2d(-WHEEL_DIST_FROM_CENTER_METERS, 
                                                                             WHEEL_DIST_FROM_CENTER_METERS);
    private final Translation2d backRightWheelLocation  = new Translation2d(-WHEEL_DIST_FROM_CENTER_METERS, 
                                                                            -WHEEL_DIST_FROM_CENTER_METERS);                                                                       

    private SwerveDriveKinematics kinematics;
                                                                            
    public Drive() {
        frontLeft =  new Wheel(FRONT_LEFT_DRIVE_CAN_ID,  FRONT_LEFT_ROTATE_CAN_ID, true);
        frontRight = new Wheel(FRONT_RIGHT_DRIVE_CAN_ID, FRONT_RIGHT_ROTATE_CAN_ID, false);
        backLeft   = new Wheel(BACK_LEFT_DRIVE_CAN_ID,   BACK_LEFT_ROTATE_CAN_ID, true);
        backRight  = new Wheel(BACK_RIGHT_DRIVE_CAN_ID,  BACK_RIGHT_ROTATE_CAN_ID, false);

        kinematics = new SwerveDriveKinematics(frontLeftWheelLocation, frontRightWheelLocation, 
                                               backLeftWheelLocation,  backRightWheelLocation);
        
        //set up gyro
        gyro = new Navx(GYRO_CAN_ID,100);
        gyro.enable9DYaw(false);
        gyro.enableOptionalMessages(
            true,
            false,
            true,
            false,
            false,
            false,
            true,
            false,
            false,
            false
        );
        gyro.resetYaw();
    }



    public void teleopDrive(double fwdPower, double strafePower, double rotatePower, boolean fieldDrive) { 
        SwerveModuleState[] swerveModuleStates;
        ChassisSpeeds       chassisSpeeds;

        if (fieldDrive == false){
            chassisSpeeds = new ChassisSpeeds(fwdPower, strafePower, rotatePower);
        }
        else {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwdPower, strafePower, rotatePower, getGyroYaw());
        }

        swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 1.0);

        frontLeft.setDesiredState (swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState  (swerveModuleStates[2]);
        backRight.setDesiredState (swerveModuleStates[3]);

    }



    public void wheelLock() {
        Rotation2d flRotation = Rotation2d.fromDegrees(45);
        Rotation2d frRotation = Rotation2d.fromDegrees(-45);
        Rotation2d blRotation = Rotation2d.fromDegrees(-45);
        Rotation2d brRotation = Rotation2d.fromDegrees(45);

        SwerveModuleState flState = new SwerveModuleState(0.0, flRotation);
        SwerveModuleState frState = new SwerveModuleState(0.0, frRotation);
        SwerveModuleState blState = new SwerveModuleState(0.0, blRotation);
        SwerveModuleState brState = new SwerveModuleState(0.0, brRotation);

        frontLeft.setDesiredState (flState);
        frontRight.setDesiredState(frState);
        backLeft.setDesiredState  (blState);
        backRight.setDesiredState (brState);
    }



    public void resetGyro() {
         gyro.resetYaw();
    }



    private Rotation2d getGyroYaw() {
       Angle  angle = gyro.getYaw();
       Double angleRadians = angle.in(Radians);
       Rotation2d yawRotation = new Rotation2d(angleRadians);
       return yawRotation;
    }

}
