package frc.robot.constants;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;

public class ConstLEDs {
  public static final ColorFlowAnimation READY_TO_SHOOT_ANIMATION = new ColorFlowAnimation(0, 185)
        .withSlot(0)
        .withColor(new RGBWColor(0, 0, 255, 0))
        .withDirection(AnimationDirectionValue.Forward)
        .withFrameRate(Hertz.of(25));

    public static final ColorFlowAnimation NONE_ANIMATION = new ColorFlowAnimation(0, 0)
        .withSlot(1)
        .withColor(new RGBWColor(0, 0, 0, 0))
        .withDirection(AnimationDirectionValue.Forward)
        .withFrameRate(Hertz.of(25));
        
    public static final ColorFlowAnimation SHIFT_GONNA_END_ANIMATION = new ColorFlowAnimation(0, 185)
        .withSlot(3)
        .withColor(new RGBWColor(255, 0, 0, 0))
        .withDirection(AnimationDirectionValue.Forward)
        .withFrameRate(Hertz.of(25));

}
