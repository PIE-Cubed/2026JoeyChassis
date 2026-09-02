// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;


/** Add your docs here. */
public class Wheel {
    private SparkFlex driveMotor;
    private SparkFlexConfig driveMotorConfig;
    private RelativeEncoder driveEncoder;
    private EncoderConfig driveEncoderConfig;
    private SparkMax rotateMotor;
    private SparkMaxConfig rotateMotorConfig;
    private SparkAbsoluteEncoder rotateEncoder;
    private AbsoluteEncoderConfig rotateEncoderConfig;

    private PIDController rotatePIDController;
    
    public static final double EXTERNAL_GEARING = 5.08;
    public static final double WHEEL_DIAMETER_INCHES = 2.88;
    private final double WHEEL_CIRCUMFERENCE_FEET =
        (Math.PI * WHEEL_DIAMETER_INCHES) / 12; // Feet per rotation (circumference)
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(
        WHEEL_DIAMETER_INCHES
    );
    public static final double NEO_VORTEX_FREE_RPM = 6500.0 * 0.9; // multiply by 0.9 for practicality
    public static final double MAX_DRIVE_VEL_MPS =
        (((WHEEL_DIAMETER_METERS * Math.PI) / EXTERNAL_GEARING) * NEO_VORTEX_FREE_RPM) /
        60;
    // MAX_DRIVE_VEL_MPS is around 4.5 meters per second
    public static final double MAX_ROTATE_VEL_DPS = 280;
    // MAX_ROTATE_VEL_DPS is around 280 degrees per second
    public static final double REV_SPIKED_WHEEL_COF = 1.2;

    // Drive Motor Conversion Factors
    private final double DRIVE_POS_CONVERSION_FACTOR =
        WHEEL_CIRCUMFERENCE_FEET / EXTERNAL_GEARING; // Feet per tick
    private final double DRIVE_VEL_CONVERSION_FACTOR = DRIVE_POS_CONVERSION_FACTOR / 60.0; // Feet per second

    // Rotate Motor Conversion Factor
    private final double ROTATE_ENCODER_CONVERSION = 360; // Convert the rotate motor's encoder to degrees

    //PID THINGy
    private final double ROTATE_P = 0.0085;
    private final double ROTATE_I = 0.0;
    private final double ROTATE_D = 0.0;

    //private final double ROTATE_PID_TOLERANCE = 0.0
    
    public Wheel(int driveID, int rotateID, boolean invertDriveMotor) {
        driveMotor  = new SparkFlex(driveID, MotorType.kBrushless);
        driveMotorConfig = new SparkFlexConfig();
        driveMotorConfig.smartCurrentLimit(Robot.VORTEX_CURRENT_LIMIT);
        driveMotorConfig.idleMode(IdleMode.kBrake);
        driveMotorConfig.inverted(invertDriveMotor);

        rotateMotor = new SparkMax(rotateID, MotorType.kBrushless);
        rotateMotorConfig = new SparkMaxConfig();
        // Same thing as the drive motor configs calls
        rotateMotorConfig
            .smartCurrentLimit(Robot.NEO_550_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        driveEncoderConfig = new EncoderConfig();
        driveEncoderConfig.positionConversionFactor(Units.feetToMeters(DRIVE_POS_CONVERSION_FACTOR));
        driveEncoderConfig.velocityConversionFactor(Units.feetToMeters(DRIVE_VEL_CONVERSION_FACTOR));

        driveMotorConfig.apply(driveEncoderConfig);

        driveMotor.configure(
            driveMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        rotateEncoder = rotateMotor.getAbsoluteEncoder();
        rotateEncoderConfig = new AbsoluteEncoderConfig();
        rotateEncoderConfig
            .positionConversionFactor(ROTATE_ENCODER_CONVERSION)
            .inverted(true);
        rotateMotorConfig.apply(rotateEncoderConfig);

        rotatePIDController = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);
        rotatePIDController.enableContinuousInput(0, 360);
        rotatePIDController.reset();
        //rotatePIDController.setTolerence(ROTATE_PID_TOLERANCE);
        rotateMotor.configure(
            rotateMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        
    }

     

    public void setDesiredState(SwerveModuleState swerveModuleState) {

        swerveModuleState.optimize(
            new Rotation2d(
                MathUtil.angleModulus(
                    Units.degreesToRadians(rotateEncoder.getPosition())
                )
            )
        );

        double currentAngleDegrees;
        double targetAngleDegrees;


        currentAngleDegrees = rotateEncoder.getPosition();
        targetAngleDegrees  = swerveModuleState.angle.getDegrees();

        double rotatePower  = rotatePIDController.calculate(currentAngleDegrees, targetAngleDegrees);

        driveMotor.set(MathUtil.clamp(swerveModuleState.speedMetersPerSecond, -1.0, 1.0));
        rotateMotor.set(MathUtil.clamp(rotatePower, -1.0, 1.0));
    }




    public SwerveModulePosition getSwerveModulePositions() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d(MathUtil.angleModulus(Units.degreesToRadians(rotateEncoder.getPosition())))
        );
    }



    /***************************************************************
                            TEST PROGRAMS
    ***************************************************************/

    public void testRotatePID(double targetAngleDegrees) {
        double currentAngleDegrees;
        
        currentAngleDegrees = rotateEncoder.getPosition();

        double rotatePower  = rotatePIDController.calculate(currentAngleDegrees, targetAngleDegrees);

        rotateMotor.set(MathUtil.clamp(rotatePower, -1.0, 1.0));
    }
}
