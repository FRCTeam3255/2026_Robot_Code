package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;

public class ConstAuto {
  public static final Time PREP_SHOOT_TIMEOUT = Units.Seconds.of(1.0);
  public static final Time SHOOT_PRELOAD_TIMEOUT = Units.Seconds.of(3.0);
  public static final Time SHOOT_DEFAULT_TIMEOUT = Units.Seconds.of(3.0);

  public static final Time SHOOT_FROM_DEPOT_TIMEOUT = Units.Seconds.of(3.0);
  public static final Time INTAKE_DEPOT_TIMEOUT = Units.Seconds.of(4.0);

  public static final Time SHOOT_FROM_OUTPOST_TIMEOUT = Units.Seconds.of(6.0);
  public static final Time INTAKE_OUTPOST_TIMEOUT = Units.Seconds.of(3.5);

  public static final Time INTAKE_NEUTRAL_ZONE_TIMEOUT = Units.Seconds.of(3.0);
  public static final Time SHOOT_NEUTRAL_ZONE_TIMEOUT = Units.Seconds.of(5.0);
}
