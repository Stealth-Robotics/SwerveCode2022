package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Drivebase extends SubsystemBase {

    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(RobotMap.DriveBase.PIGEON_IMU);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public Drivebase() {
        ShuffleboardTab tab = Shuffleboard.getTab("DriveBase");

        // pigeon.configFactoryDefault();
        setGyroscopeRotation(90);

        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                RobotMap.DriveBase.FRONT_LEFT_MODULE_DRIVE,
                RobotMap.DriveBase.FRONT_LEFT_MODULE_STEER,
                RobotMap.DriveBase.FRONT_LEFT_MODULE_STEER_ENCODER,
                Constants.DriveBase.FRONT_LEFT_MODULE_STEER_OFFSET);

        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                RobotMap.DriveBase.FRONT_RIGHT_MODULE_DRIVE,
                RobotMap.DriveBase.FRONT_RIGHT_MODULE_STEER,
                RobotMap.DriveBase.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Constants.DriveBase.FRONT_RIGHT_MODULE_STEER_OFFSET);

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                RobotMap.DriveBase.BACK_LEFT_MODULE_DRIVE,
                RobotMap.DriveBase.BACK_LEFT_MODULE_STEER,
                RobotMap.DriveBase.BACK_LEFT_MODULE_STEER_ENCODER,
                Constants.DriveBase.BACK_LEFT_MODULE_STEER_OFFSET);

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                RobotMap.DriveBase.BACK_RIGHT_MODULE_DRIVE,
                RobotMap.DriveBase.BACK_RIGHT_MODULE_STEER,
                RobotMap.DriveBase.BACK_RIGHT_MODULE_STEER_ENCODER,
                Constants.DriveBase.BACK_RIGHT_MODULE_STEER_OFFSET);
    }

    public Rotation2d getGyroscopeRotation() {
        return pigeon.getRotation2d();
    }

    public void setGyroscopeRotation(double newValue) {
        pigeon.setYaw(90);
    }

    public void zeroGyroscope() {
        setGyroscopeRotation(0);
    }

    public void drive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        this.chassisSpeeds = new ChassisSpeeds(
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.set(
                desiredStates[0].speedMetersPerSecond
                        / Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND
                        * Constants.DriveBase.MAX_VOLTAGE,
                desiredStates[0].angle.getRadians());
        frontRightModule.set(
                desiredStates[1].speedMetersPerSecond
                        / Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND
                        * Constants.DriveBase.MAX_VOLTAGE,
                desiredStates[1].angle.getRadians());
        backLeftModule.set(
                desiredStates[2].speedMetersPerSecond
                        / Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND
                        * Constants.DriveBase.MAX_VOLTAGE,
                desiredStates[2].angle.getRadians());
        backRightModule.set(
                desiredStates[3].speedMetersPerSecond
                        / Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND
                        * Constants.DriveBase.MAX_VOLTAGE,
                desiredStates[3].angle.getRadians());
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = Constants.DriveBase.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
    }
}
