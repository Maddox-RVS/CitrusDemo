package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;

public class SwerveCommands {
    public static Command commandXDrives(final Swerve swerve) {
        return swerve.runOnce(() -> {
            SwerveModuleState[] newModuleStates = {
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(270)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(180)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(90)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(0))
            };
            swerve.setModuleStates(newModuleStates);
        }).withName("commandXDrives");
    }

    public static Command commandStopDrives(final Swerve swerve) {
        return swerve.runOnce(() -> swerve.setModuleStates(new ChassisSpeeds())).withName("commandStopDrives");
    }

    public static Command commandFlipGyro() {
        return RobotContainer.swerve.runOnce(
            () -> RobotContainer.swerve.setYaw(
                RobotContainer.swerve.getYawRot().rotateBy(Rotation2d.fromDegrees(180)).getDegrees()
            )
        ).withName("commandFlipGyro");
    }
}
