package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private Pigeon2 gyro;

    public Swerve() {
        this.gyro = new Pigeon2(Constants.kSwerve.PIGEON_ID, Constants.kSwerve.CANBUS);
        var gyroEmptyConfig = new Pigeon2Configuration();
        gyro.getConfigurator().apply(gyroEmptyConfig);
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(Constants.kSwerve.Mod0.CONSTANTS, 0),
                new SwerveModule(Constants.kSwerve.Mod1.CONSTANTS, 1),
                new SwerveModule(Constants.kSwerve.Mod2.CONSTANTS, 2),
                new SwerveModule(Constants.kSwerve.Mod3.CONSTANTS, 3)
        };

        Timer.delay(1.0);

        swerveOdometry = new SwerveDriveOdometry(Constants.kSwerve.SWERVE_KINEMATICS, getYaw(), getModulePositions());
    }

    public void Drive(Translation2d translation, double rotation, boolean fieldRelative) {
        SwerveModuleState[] mSwerveModuleStates = Constants.kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(mSwerveModuleStates, Constants.kSwerve.MAX_SPEED);

        for (SwerveModule module : mSwerveMods) {
            module.setDesiredState(mSwerveModuleStates[module.moduleNumber]);
        }
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    public Rotation2d getYaw() {
        return (Constants.kSwerve.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValue())
                : Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (SwerveModule module : mSwerveMods) {
            modulePositions[module.moduleNumber] = module.getPosition();
        }
        return modulePositions;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kSwerve.MAX_SPEED);

        for (SwerveModule module : mSwerveMods) {
            module.setDesiredState(desiredStates[module.moduleNumber]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : mSwerveMods) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        for (SwerveModule module : mSwerveMods) {
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Cancoder", module.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Angle",
                    module.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Velocity",
                    module.getState().speedMetersPerSecond);
        }
    }
}
