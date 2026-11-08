package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "Blue Auto", group = "Auto", preselectTeleOp = "Blue Teleop")
public class BlueAuto extends AutoBase {
    public BlueAuto() {
        super(Alliance.BLUE);
    }
}
