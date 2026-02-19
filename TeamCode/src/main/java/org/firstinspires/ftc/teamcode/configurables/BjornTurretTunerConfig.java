package org.firstinspires.ftc.teamcode.configurables;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

/**
 * Configuration for the BjornTurretAutoTuner (Live Mode).
 */
@Configurable
public class BjornTurretTunerConfig {

    @Sorter(sort = 0)
    public static boolean runTest = false; // Toggle to start/stop oscillation

    @Sorter(sort = 1)
    public static double targetLow = 0.0;

    @Sorter(sort = 2)
    public static double targetHigh = 90.0;

    @Sorter(sort = 3)
    public static double periodSec = 4.0;

    @Sorter(sort = 4)
    public static double manualTarget = 90.0;

    @Sorter(sort = 5)
    public static double counterRotationGain = 0.002;
}
