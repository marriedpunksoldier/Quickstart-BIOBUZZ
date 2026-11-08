package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "Red Auto", group = "Auto", preselectTeleOp = "Red Teleop")
public class RedAuto extends AutoBase {
    public RedAuto() {
        super(Alliance.RED);
    }
}
