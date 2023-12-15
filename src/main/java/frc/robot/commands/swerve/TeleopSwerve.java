package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;

/** An example command that uses an example subsystem. */
public class TeleopSwerve extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Swerve swerve;
    private final DoubleSupplier translationXSup;
    private final DoubleSupplier translationYSup;
    private final DoubleSupplier rotationAxisSup;

    public TeleopSwerve(Swerve swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationXSup = translation;
        this.translationYSup = strafe;
        this.rotationAxisSup = rotation;
    }

    @Override
    public void execute() {
        double translationVal;
        double strafeVal;
        double rotationVal;
        double swerveTranslationMultiplier = 
            NetworkTableInstance.getDefault().getEntry("/Shuffleboard/Driver/Swerve Translation Multiplier")
            .getDouble(kSwerve.SWERVE_DEFAULT_TRANSLATION);
        double swerveRotationMultiplier = 
            NetworkTableInstance.getDefault().getEntry("/Shuffleboard/Driver/Swerve Rotation Multiplier")
            .getDouble(kSwerve.SWERVE_DEFAULT_ROTATION);

        translationVal = MathUtil.applyDeadband(
            -translationXSup.getAsDouble(), 
            ControllerConsts.LEFT_DEADBAND) * swerveTranslationMultiplier;
        strafeVal = MathUtil.applyDeadband(
            -translationYSup.getAsDouble(), 
            ControllerConsts.LEFT_DEADBAND) * swerveTranslationMultiplier;
        rotationVal = MathUtil.applyDeadband(
            rotationAxisSup.getAsDouble(), 
            ControllerConsts.RIGHT_DEADBAND) * swerveRotationMultiplier;

        swerve.drive(
                new Translation2d(translationVal, strafeVal)
                    .times(Constants.kSwerve.MAX_SPEED),
                rotationVal * Constants.kSwerve.MAX_ANGULAR_VELOCITY,
                true,
                true);
    }
}
