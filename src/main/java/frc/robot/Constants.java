// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static final class DriveBase {

                // TODO: Tune To Your Robot
                public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(129.72);
                public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(116.098);
                public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(61.688);
                public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(245.558);

                // TODO: Tune To Your Robot
                public static final double TRACKWIDTH = Units.inchesToMeters(23.5);
                public static final double WHEELBASE = Units.inchesToMeters(23.125);

                // TODO: Tune TO Your Robot
                public static final double SPEED_MULTIPLIER = 1;

                public static final double MAX_VOLTAGE = 12.0;

                public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                                SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

                public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                                Math.hypot(TRACKWIDTH / 2.0, WHEELBASE / 2.0);

                public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                                // Front left
                                new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                                // Front right
                                new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                                // Back left
                                new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                                // Back right
                                new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0));

        }
}
