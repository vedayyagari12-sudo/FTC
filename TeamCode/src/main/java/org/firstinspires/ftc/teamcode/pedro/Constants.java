package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; // needed by AutoTune's Pinpoint output

import java.util.OptionalDouble; // needed if you ever use a Custom pod type

/**
 * Pedro Pathing 3 constants for the BIOBUZZ drivetrain.
 *
 * Three blocks below get REPLACED by AutoTune output (open the "Java" tab in AutoTune and
 * paste over the whole block, from "public static ..." to the closing ");"):
 *   1) drivetrainConfig  <- Mecanum Procedure
 *   2) localizerConfig   <- Pinpoint Procedure
 *   3) foresightConfig   <- Foresight Procedure
 *
 * See PEDRO_TUNING_CHECKLIST.md in the project root for the full step-by-step.
 */
public class Constants {

    // ===================== 1) DRIVETRAIN  (paste Mecanum Procedure output here) =====================
    // CONFIRMED by AutoTune Mecanum Procedure (2026-10-11). Names match physical wheel positions.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("FrontLeft");
                c.backLeftName.set("BackLeft");
                c.frontRightName.set("FrontRight");
                c.backRightName.set("BackRight");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                // Pasted AutoTune output will not include this line - keep it so teleop holds with brake.
                c.manualBrakeMode.set(true);
            }
    );

    // ===================== 2) LOCALIZER  (paste Pinpoint Procedure output here) =====================
    // goBILDA Swing Arm pods, Pinpoint named "pinpoint" on Control Hub I2C port 1.
    // Offsets measured by hand. Localization test: x goes up pushing forward, y goes up pushing left,
    // and x/y stay within about 5 in during a full spin (2026-10-17).
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
                c.xPodOffset.set(-10.5); // measured by hand: forward pod ~10.5 in RIGHT of center (negative = right)
                c.yPodOffset.set(0.0); // measured by hand: sideways pod exactly centered front-to-back
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED); // verified by Localization test: x goes UP when pushed forward
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
                c.offsetUnits.set(DistanceUnit.INCH);
            }
    );

    // ===================== 3) FORESIGHT  (paste Foresight Procedure output here) =====================
    // From AutoTune Foresight Procedure (2026-10-17).
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.17215546286376668);
                Controller secondaryTranslationalForward = Controller.proportional(0.06360684344947322);
                Controller primaryTranslationalLateral = Controller.proportional(0.24481465113278023);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09045247202560626);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011629575145250464));
                c.brake.set(Controller.proportionalFeedforward(0.009885138873462894));

                c.headingFeedback.set(Controller.proportional(3.9126062932681482));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0493331289074688, 0.004029512834442165));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06891287392553444, 0.055308920497519294));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.561435437875014E-4, 0.0010238098946001462));

                c.maxAchievableForwardVelocity.set(89.93791309249306);
                c.maxAchievableStrafeVelocity.set(62.458093122155205);
                c.naturalForwardDeceleration.set(51.77627195693382);
                c.naturalStrafeDeceleration.set(80.87165590642635);
            }
    );

    // ===================== Follower factory (do not edit) =====================
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
//