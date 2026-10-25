package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Procedures that appear in AutoTune (http://192.168.43.1:10158 while connected to the robot).
 * Run them in this order: Mecanum -> Pinpoint -> Foresight -> Tests.
 */
public class Tuning {

    /** 1. Finds motor directions and generates drivetrainConfig. */
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    /** 2. Finds Pinpoint pod directions + offsets and generates localizerConfig. */
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    /** 3. Generates foresightConfig (velocities, braking, kP values). */
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    /** 4. Driving / Localization / Line / Curve / Hold tests to verify everything. */
    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig));
    }
}
