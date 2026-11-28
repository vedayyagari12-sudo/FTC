package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Blue TeleOp - exactly RedTeleOp (same controls, hardware and shooter); only the distance readout uses the
 * BLUE HIVE (x = 84.75 in TurretAim). Change behaviour in RedTeleOp and both alliances get it.
 */
@TeleOp(name = "Blue TeleOp")
public class BlueTeleOp extends RedTeleOp {
    @Override
    int alliance() { return TurretAim.BLUE; }
}
